#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <crazyflie_controller/GenericLogData.h>

#include "pid.hpp"

double get(
    const ros::NodeHandle& n,
    const std::string& name) {
    double value;
    n.getParam(name, value);
    return value;
}

/*----Added global variables----*/
static float mass=0.0307;
static float grav=9.81;


class Controller
{
public:

    Controller(
        const std::string& worldFrame,
        const std::string& frame,
        const ros::NodeHandle& n)
        : m_worldFrame(worldFrame)
        , m_frame(frame)
        , m_pubNav()
        , m_listener()
        , m_pidX(
            get(n, "PIDs/X/kp"),
            get(n, "PIDs/X/kd"),
            get(n, "PIDs/X/ki"),
            get(n, "PIDs/X/minOutput"),
            get(n, "PIDs/X/maxOutput"),
            get(n, "PIDs/X/integratorMin"),
            get(n, "PIDs/X/integratorMax"),
            "x")
        , m_pidY(
            get(n, "PIDs/Y/kp"),
            get(n, "PIDs/Y/kd"),
            get(n, "PIDs/Y/ki"),
            get(n, "PIDs/Y/minOutput"),
            get(n, "PIDs/Y/maxOutput"),
            get(n, "PIDs/Y/integratorMin"),
            get(n, "PIDs/Y/integratorMax"),
            "y")
        , m_pidZ(
            get(n, "PIDs/Z/kp"),
            get(n, "PIDs/Z/kd"),
            get(n, "PIDs/Z/ki"),
            get(n, "PIDs/Z/minOutput"),
            get(n, "PIDs/Z/maxOutput"),
            get(n, "PIDs/Z/integratorMin"),
            get(n, "PIDs/Z/integratorMax"),
            "z")
        , m_pidYaw(
            get(n, "PIDs/Yaw/kp"),
            get(n, "PIDs/Yaw/kd"),
            get(n, "PIDs/Yaw/ki"),
            get(n, "PIDs/Yaw/minOutput"),
            get(n, "PIDs/Yaw/maxOutput"),
            get(n, "PIDs/Yaw/integratorMin"),
            get(n, "PIDs/Yaw/integratorMax"),
            "yaw")
        , m_state(Idle)
        , m_goal()
        , m_subscribeGoal()
        , m_serviceTakeoff()
        , m_serviceLand()
        , m_thrust(0)
        , m_startZ(0)
    {
        ros::NodeHandle nh;
        m_listener.waitForTransform(m_worldFrame, m_frame, ros::Time(0), ros::Duration(10.0)); 
        m_pubNav = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        m_subscribeGoal = nh.subscribe("goal", 1, &Controller::goalChanged, this);
        m_serviceTakeoff = nh.advertiseService("takeoff", &Controller::takeoff, this);
        m_serviceLand = nh.advertiseService("land", &Controller::land, this);

        //Topic added to the original one.
        sub=nh.subscribe("vrpn_client_node/cf1/pose", 1, &Controller::getPose, this);
        sub3=nh.subscribe("imu", 1, &Controller::getAccel, this);
        sub4=nh.subscribe("log2", 1, &Controller::getThrust, this);
        sub5=nh.subscribe("log1", 1, &Controller::getParameters, this);
        pub1=nh.advertise<geometry_msgs::Twist>("Tension", 1);
        pub2=nh.advertise<geometry_msgs::PoseStamped>("New_goal", 1);
    }

    void run(double frequency)
    {
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &Controller::iteration, this);
        ros::spin();
    }

private:
    void goalChanged(const geometry_msgs::PoseStamped::ConstPtr& msg){
    m_goal = *msg;
    geometry_msgs::PoseStamped m_goal2;
    }

    bool takeoff(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
        ROS_INFO("Takeoff requested!");
        m_state = TakingOff;

        tf::StampedTransform transform;
        m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
        m_startZ = transform.getOrigin().z();

        return true;
    }

    bool land(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
        ROS_INFO("Landing requested!");
        m_state = Landing;

        return true;
    }

    void getTransform(
        const std::string& sourceFrame,
        const std::string& targetFrame,
        tf::StampedTransform& result)
    {
        m_listener.lookupTransform(sourceFrame, targetFrame, ros::Time(0), result);
    }

    void pidReset()
    {
        m_pidX.reset();
        m_pidY.reset();
        m_pidZ.reset();
        m_pidYaw.reset();
    }

    /*---------Added function--------*/    
    void getPose(const geometry_msgs::PoseStamped::ConstPtr& msg){
        /*Gets the current pose*/
        act_pose=*msg;
                
        //Get current attitude
        tf::Matrix3x3(
                    tf::Quaternion(
                        act_pose.pose.orientation.x,
                        act_pose.pose.orientation.y,
                        act_pose.pose.orientation.z,
                        act_pose.pose.orientation.w
                    )).getRPY(roll_m, pitch_m, yaw_m);  
        estimate_tension();
    }

    void getParameters(const crazyflie_controller::GenericLogData::ConstPtr& msg){
        /*Used to get the tension estimate using the onboard sensors instead of the 
        mocap system. Uncomment the "estimate_tension()" statement and replace the 
        angles variables in the "estimate_tension" function by these ones.
        */
        crazyflie_controller::GenericLogData m_msg=*msg;

        m_rollImu=m_msg.values[0];
        m_pitchImu=m_msg.values[1];
        m_yawImu=m_msg.values[2];

        //estimate_tension();          
    }

    void getThrust(const crazyflie_controller::GenericLogData::ConstPtr& msg){
        /*Gets the PWM values for the thrust of each motor and outputs the total thrust as being 
        the sum of all them (after mapping them to Newton - force).
        */
        crazyflie_controller::GenericLogData mens;
        mens=*msg;
        m_PWM1=mens.values[0];
        m_PWM2=mens.values[1];
        m_PWM3=mens.values[2];
        m_PWM4=mens.values[3];
        float t1=2.130295*pow(10,-11)*pow(m_PWM1,2)+1.032633*pow(10, -6)*m_PWM1+5.484560*pow(10,-4);
        float t2=2.130295*pow(10,-11)*pow(m_PWM2,2)+1.032633*pow(10, -6)*m_PWM2+5.484560*pow(10,-4);
        float t3=2.130295*pow(10,-11)*pow(m_PWM3,2)+1.032633*pow(10, -6)*m_PWM3+5.484560*pow(10,-4);
        float t4=2.130295*pow(10,-11)*pow(m_PWM4,2)+1.032633*pow(10, -6)*m_PWM4+5.484560*pow(10,-4);
        Fp=t1+t2+t3+t4;
    }
    void getAccel(const sensor_msgs::Imu::ConstPtr& msg){
        /*Gets the acceleration from the onboard sensors*/
        sensor_msgs::Imu mens;
        mens=*msg;

        ax=mens.linear_acceleration.x;
        ay=mens.linear_acceleration.y;
        az=mens.linear_acceleration.z;
    }

    void estimate_tension(){
        /*This function presents the indirect measurements for the tension applied to the quadcopter,
        which is later on used as the observation of a simple Kalman filter with constant model.     
        */
        geometry_msgs::Twist msg;

        float a=cos(yaw_m)*cos(pitch_m);
        float b=-sin(yaw_m)*cos(roll_m)+cos(yaw_m)*sin(pitch_m)*sin(roll_m);
        float c=sin(yaw_m)*sin(roll_m)+cos(yaw_m)*sin(pitch_m)*cos(roll_m);
        
        float d=sin(yaw_m)*cos(pitch_m);
        float e=cos(yaw_m)*cos(roll_m)+sin(yaw_m)*sin(pitch_m)*sin(roll_m);
        float f=-cos(yaw_m)*sin(roll_m)+sin(yaw_m)*sin(pitch_m)*cos(roll_m);

        float g=-sin(pitch_m);
        float h=cos(pitch_m)*sin(roll_m);
        float i=cos(pitch_m)*cos(roll_m);

        //if the working with too small tensions or if the tension threshold is too small remove the acceleration term, use only the attitude and thrust 
        //(in that case don't forget to add the gravity constant in Tz expression).
        Tx=c*Fp-mass*(a*ax+b*ay+c*az);
        Ty=f*Fp-mass*(d*ax+e*ay+f*az);
        Tz=i*Fp-mass*(g*ax+h*ay+i*az);

        mod_T=sqrt(pow(Tx,2)+pow(Ty,2)+pow(Tz,2));
        
        kalman();

        /*Too see the tension along the x, y and z directions. Kalman function cannot publish at the same time, same topic is being used.
        msg.linear.x=Tx;
        msg.linear.y=Ty;
        msg.linear.z=Tz;
        pub1.publish(msg);
        */
    }

    void kalman(){
        /*Kalman filter assuming a constant model to estimate the tension applied to the quadcopter. 
        Efficient model if the quadcopter is not abruptly pulled.
        */
        geometry_msgs::Twist msg;

        float Q=1;
        float R=400;

        //Update part
        T2=T2;
        P=P+Q;

        //Correction part
        float K=P/(P+R);
        T2=T2+K*(mod_T-T2);
        P=(1-K)*P;

        //Allows the visuallization of the tension estimate before and after the Kalman filtering.
        msg.angular.y=mod_T;
        msg.angular.z=T2;
        pub1.publish(msg);

    }
    /*-----end of the added function-----*/

    void iteration(const ros::TimerEvent& e)
    {
        float dt = e.current_real.toSec() - e.last_real.toSec();

        switch(m_state)
        {
        case TakingOff:
            {
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
                if (transform.getOrigin().z() > m_startZ + 0.05 || m_thrust > 50000)
                {
                    pidReset();
                    m_pidZ.setIntegral(m_thrust / m_pidZ.ki());
                    m_state = Automatic;
                    m_thrust = 0;
                }
                else
                {
                    m_thrust += 10000 * dt;
                    geometry_msgs::Twist msg;
                    msg.linear.z = m_thrust;
                    m_pubNav.publish(msg);
                }

            }
            break;
        case Landing:
            {
                m_goal.pose.position.z = m_startZ + 0.05;
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
                if (transform.getOrigin().z() <= m_startZ + 0.05) {
                    m_state = Idle;
                    geometry_msgs::Twist msg;
                    m_pubNav.publish(msg);
                }
            }
            
        case Automatic:
            {
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);

                geometry_msgs::PoseStamped targetWorld;
                targetWorld.header.stamp = transform.stamp_;
                targetWorld.header.frame_id = m_worldFrame;
                


                /*------TENSION FOLLOWING FEATURE: Changes the goal position of the quadcopter if the tension estimate is bigger than a given threshold-----*/
                
                //To achieve the first original position without using the tension following feature
                if(sqrt(pow((m_goal.pose.position.x-act_pose.pose.position.x),2)+pow((m_goal.pose.position.y-act_pose.pose.position.y),2)+
                    pow((m_goal.pose.position.z-act_pose.pose.position.z),2))>0.1 && flag==0){
                    new_goal=m_goal;
                }
                else{
                    flag=1;
                }
                
                //Updates the goal position to be the current position, threshold of 0.04N (can be changed)
                if(abs(T2)>=0.04 && flag==1){
                    new_goal=act_pose;
                }
                //The goal attitude must always be null (if using yaw rate instead of yaw angle then the yaw rate must be null)
                new_goal.pose.orientation.x=0;
                new_goal.pose.orientation.y=0;
                new_goal.pose.orientation.z=0;
                new_goal.pose.orientation.w=1;

                targetWorld.pose=new_goal.pose;                
                //To use without the tension following feature uncomment next line and comment the previous one
                //targetWorld.pose=m_goal.pose;

                //Allows to see the "new_goal" values on the topic "New_goal"
                pub2.publish(new_goal);

                /*-----end of the tension following feature-----*/

                geometry_msgs::PoseStamped targetDrone;
                m_listener.transformPose(m_frame, targetWorld, targetDrone);

                tfScalar roll, pitch, yaw;
                tf::Matrix3x3(
                    tf::Quaternion(
                        targetDrone.pose.orientation.x,
                        targetDrone.pose.orientation.y,
                        targetDrone.pose.orientation.z,
                        targetDrone.pose.orientation.w
                    )).getRPY(roll, pitch, yaw);

               
                geometry_msgs::Twist msg;
                msg.linear.x = m_pidX.update(0, targetDrone.pose.position.x);
                msg.linear.y = m_pidY.update(0.0, targetDrone.pose.position.y);
                msg.linear.z = m_pidZ.update(0.0, targetDrone.pose.position.z);
                msg.angular.z = m_pidYaw.update(0.0, yaw);


                //Added feature that turns off the motors if the distance to the ground is smaller than 0,1 meter (can be changed)
                if(flag==1 and act_pose.pose.position.z<0.1){
                    msg.linear.z=0;
                }

                m_pubNav.publish(msg);


            }
            break;
        case Idle:
            {
                geometry_msgs::Twist msg;
                m_pubNav.publish(msg);
            }
            break;
        }
    }

private:

    enum State
    {
        Idle = 0,
        Automatic = 1,
        TakingOff = 2,
        Landing = 3,
    };

private:
    std::string m_worldFrame;
    std::string m_frame;
    ros::Publisher m_pubNav;
    tf::TransformListener m_listener;
    PID m_pidX;
    PID m_pidY;
    PID m_pidZ;
    PID m_pidYaw;
    State m_state;
    geometry_msgs::PoseStamped m_goal;
    ros::Subscriber m_subscribeGoal;
    ros::ServiceServer m_serviceTakeoff;
    ros::ServiceServer m_serviceLand;
    float m_thrust;
    float m_startZ;

    ros::Subscriber sub, sub2;
    geometry_msgs::PoseStamped act_pose;

    /*----Added variables-----*/  
    float m_rollImu, m_pitchImu, m_yawImu, m_PWM1, m_PWM2, m_PWM3, m_PWM4;
    geometry_msgs::PoseStamped new_goal;
    float ax, ay, az, Fp, mod_T, Tx, Ty, Tz;
    double roll_m, pitch_m, yaw_m;
    ros::Subscriber sub3, sub4, sub5;
    ros::Publisher pub1, pub2, pub3;
    int flag=0;
    float T2=0;
    float P=1;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");

  // Read parameters
  ros::NodeHandle n("~");
  std::string worldFrame;
  n.param<std::string>("worldFrame", worldFrame, "/world");
  std::string frame;
  n.getParam("frame", frame);
  double frequency;
  n.param("frequency", frequency, 50.0);

  Controller controller(worldFrame, frame, n);
  controller.run(frequency);

  return 0;
}
