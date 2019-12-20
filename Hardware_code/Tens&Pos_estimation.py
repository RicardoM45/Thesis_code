import rospy
import matplotlib.pyplot as plt
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped, Twist, Quaternion
from sensor_msgs.msg import Imu 
from math import cos, sin, pi, sqrt, cosh, sinh, atan, asinh, atan2, acosh
from numpy.linalg import multi_dot
from numpy.linalg import inv
#To read the attitude from the on-board sensors
from crazyflie_demo.msg import GenericLogData
import datetime

mass=0.0307
grav=9.81

#Tether's lenght and weight
s_tot=1.6
w_tot=11.6

w=w_tot/(s_tot*1000)*9.81

#Coordinates of the tether's origin with respect to the world frame
z_inicial=0.754
r_inicial=1.18

#Offsets regarding the accelerometers
DaccZ=0
DaccX=0
DaccY=0

#Para limitar os plots
flag=1
i_time=0
f_time=0

#Used to compute the position errors
av_error=np.zeros(3)
count=0
max_error=np.zeros(3)
avR_error=np.zeros(3)

class wp():
	def __init__(self):
		self.accX=0
		self.accY=0
		self.accZ=0
		self.roll=0
		self.pitch=0
		self.yaw=0
		self.thrust=0
		self.rollImu=0
		self.pitchImu=0
		
		#Kalman variables
		self.T=np.transpose(np.zeros(3)+pow(10,-5))
		self.P=np.identity(3)
		self.H=np.identity(3)
		self.X=np.transpose(np.zeros(3))
		
		#Variable regarding errors
		self.t=0
		
		#Variables regarding the position estimation
		self.realPosX=list()
		self.realPosY=list()
		self.realPosZ=list()
		self.posX=0
		self.posY=0
		self.posZ=0
		self.R0=list()
		self.realR0=list()

		#Imu
		self.rollImu_vec=list()
		self.pitchImu_vec=list()

		#Confirmacao
		self.PosX2=list()
		self.PosY2=list()
		self.PosZ2=list()
		self.realPosX2=list()
		self.realPosY2=list()
		self.realPosZ2=list()


	def getAcc(self, msg):
		#Gets the acceleration from the on-board sensors
		self.accX=msg.linear_acceleration.x
		self.accY=msg.linear_acceleration.y
		self.accZ=msg.linear_acceleration.z

	def getAttitude(self, msg):
		global i_time, flag, f_time 

		quaternion=(msg.pose.orientation.x, msg.pose.orientation.y,	msg.pose.orientation.z,	msg.pose.orientation.w)
		euler=tf.transformations.euler_from_quaternion(quaternion)
		
		#Change the offset if needed, used offsets were 0.02 and -0.03. Angles in radians
		self.roll=euler[0]+0.02
		self.pitch=euler[1]-0.03
		self.yaw=euler[2] 

		#Used to print the graphics with the correct time
		if(flag==1):
			i_time=msg.header.stamp.secs
			flag=0
		f_time=msg.header.stamp.secs-i_time 
		
	def getRollPitch(self, msg):
		#One way of computing the offset is by compute the mean of the gaussian approximation of the error distribution
		#Angles in degrees
		self.rollImu=msg.values[0]#+offset: add offset if needed 
		self.pitchImu=msg.values[1]#+offset: add offset if needed
		
		self.rollImu_vec.append(self.rollImu)
		self.pitchImu_vec.append(-self.pitchImu)

	def getThrust(self, msg):
		pwm_m1=msg.values[0]
		pwm_m2=msg.values[1]
		pwm_m3=msg.values[2]
		pwm_m4=msg.values[3]

		#Converts the PWM values into thrust
		t1=2.130295*pow(10,-11)*pow(pwm_m1,2)+1.032633*pow(10, -6)*pwm_m1+5.484560*pow(10,-4)
		t2=2.130295*pow(10,-11)*pow(pwm_m2,2)+1.032633*pow(10, -6)*pwm_m2+5.484560*pow(10,-4)
		t3=2.130295*pow(10,-11)*pow(pwm_m3,2)+1.032633*pow(10, -6)*pwm_m3+5.484560*pow(10,-4)
		t4=2.130295*pow(10,-11)*pow(pwm_m4,2)+1.032633*pow(10, -6)*pwm_m4+5.484560*pow(10,-4)
		
		#If needed multiply by some offset
		self.thrust=(t1+t2+t3+t4)*1.1
		
	#------Observation part for Kalman--------
	def getTension(self, msg):
		global mass, grav, z_inicial, r_inicial, DaccZ, DaccX, DaccY

		#Used in the error computation
		self.t=self.t+1

		#To use the on-board attitude sensors uncomment
		#self.pitch=-self.pitchImu/180*PI 
		#self.roll=self.rollImu/180*PI

		a=cos(self.yaw)*cos(self.pitch)
		b=-sin(self.yaw)*cos(self.roll)+cos(self.yaw)*sin(self.pitch)*sin(self.roll)
		c=sin(self.yaw)*sin(self.roll)+cos(self.yaw)*sin(self.pitch)*cos(self.roll)

		d=sin(self.yaw)*cos(self.pitch)
		e=cos(self.yaw)*cos(self.roll)+sin(self.yaw)*sin(self.pitch)*sin(self.roll)
		f=-cos(self.yaw)*sin(self.roll)+sin(self.yaw)*sin(self.pitch)*cos(self.roll)

		g=-sin(self.pitch)
		h=cos(self.pitch)*sin(self.roll)
		i=cos(self.pitch)*cos(self.roll)

		#For non-hovering cases uncomment
		#self.Tx=mass*(a*(self.accX+DaccX)+b*(self.accY+DaccY)+c*(self.accZ+DaccZ))-c*self.thrust
		#self.Ty=mass*(d*(self.accX+DaccX)+e*(self.accY+DaccY)+f*(self.accZ+DaccZ))-f*self.thrust
		#self.Tz=mass*(g*(self.accX+DaccX)+h*(self.accY+DaccY)+i*(self.accZ+DaccZ))-i*self.thrust

		#For non-hovering cases comment
		self.Tx=-c*self.thrust
		self.Ty=-f*self.thrust
		self.Tz=mass*(9.81)-i*self.thrust

		#Converts the real position from the Mocap to the position with respect to tether's frame
		self.posX=msg.pose.position.x
		self.posY=msg.pose.position.y
		self.posZ=msg.pose.position.z
		self.realPosX2.append(r_inicial-msg.pose.position.x)
		self.realPosY2.append(-msg.pose.position.y)
		self.realPosZ2.append(msg.pose.position.z)
		
		self.kalman()

	def drawgraphic(self):
		global lim_i, lim_f

		self.time=np.linspace(0,f_time, len(self.PosX2))
		ax1=plt.subplot(3,1,1)
		ax1.plot(self.time, self.PosX2, label="Estimated X", linewidth=2)
		ax1.plot(self.time, self.realPosX2, label="Real X", linewidth=2)
		plt.title("X")
		plt.tight_layout()
		plt.legend()
		ax2=plt.subplot(3,1,2)
		ax2.plot(self.time, self.PosY2, label="Estimated Y", linewidth=2)
		ax2.plot(self.time, self.realPosY2, label="Real Y", linewidth=2)
		plt.title("Y")
		plt.tight_layout()
		plt.legend()
		ax3=plt.subplot(3,1,3)
		ax3.plot(self.time, self.PosZ2, label="Estimated Z", linewidth=2)
		ax3.plot(self.time, self.realPosZ2, label="Real Z", linewidth=2)
		plt.title("Z")
		plt.tight_layout()
		plt.legend()
		plt.show()
	
	def kalman(self):
		Q=np.diag([0.1,0.1,0.1])
		R=np.diag([400,400,400])

		#Observation vector
		Z=np.array([self.Tx, self.Ty, self.Tz])


		#Update part using a constant model
		self.T=self.T
		self.P=self.P+Q
		#Correction part
		K=multi_dot([self.P,np.transpose(self.H),inv(multi_dot([self.H,self.P,np.transpose(self.H)])+R)])
		self.T=self.T+np.dot(K, (Z-np.dot(self.H, np.transpose(self.T))))
		self.P=np.dot((np.identity(3)-K),self.P)
		
		self.estimate_position()

	def estimate_position(self):
		global w, s_tot, lim_i, lim_f, count, av_error, z_inicial, avR_error, max_error

		yi=z_inicial

		T0=sqrt(pow(self.T[0],2)+pow(self.T[1],2))
		
		#Catenary parameters estimation based on the tension applied to the UAV
		a=T0/w
		s2=abs(self.T[2]/w)
		
		teta=atan2(-self.posY, r_inicial-self.posX)
		
		#Radial distance estimate
		x0=-a*asinh((s2-s_tot)/a)
		r=a*asinh(s2/a)+x0
		
		#Altitude estimate
		C1=yi-a*cosh(-x0/a)
		z=a*cosh((r-x0)/a)+C1
		C2=z-a*cosh((r-x0)/a)
		C=(C1+C2)/2
		z4=a*cosh((r-x0)/a)+C
		
		#Decomposition of the radial distance in terms of "x" and "y" coordinates
		x=r*cos(teta)
		y=r*sin(teta)	

		self.PosX2.append(x)
		self.PosY2.append(y)
		self.PosZ2.append(z4)
		
		#Computes the average relative, the average absolute and the maximum errors
		if(self.t>=250):
			count=count+1
			#Absolute average error (in meter)
			av_error[0]=av_error[0]+abs((x-(r_inicial-self.posX)))
			av_error[1]=av_error[1]+abs((y+self.posY))
			av_error[2]=av_error[2]+abs((z4-self.posZ))
			
			#Relative error
			avR_error[0]=avR_error[0]+abs((x-(r_inicial-self.posX))/(r_inicial-self.posX))
			avR_error[1]=avR_error[1]+abs((y+self.posY)/self.posY)
			avR_error[2]=avR_error[2]+abs((z4-self.posZ)/self.posZ)

			#Maximum absolute error (in meter)
			aux1=abs((x-(r_inicial-self.posX)))
			aux2=abs((y+self.posY))
			aux3=abs((z4-self.posZ))
			if(aux1>max_error[0]):
				max_error[0]=aux1
			if(aux2>max_error[1]):
				max_error[1]=aux2
			if(aux3>max_error[2]):
				max_error[2]=aux3
			

	def run(self):
		rospy.Subscriber('crazyflie/imu', Imu, self.getAcc)	
		rospy.Subscriber('crazyflie/vrpn_client_node/cf1/pose', PoseStamped, self.getAttitude)
		rospy.Subscriber('crazyflie/vrpn_client_node/cf1/pose', PoseStamped, self.getTension)
		rospy.Subscriber('crazyflie/log2', GenericLogData, self.getThrust)
		rospy.Subscriber('crazyflie/log1', GenericLogData, self.getRollPitch)
		rospy.spin()


if __name__ == '__main__':
	rospy.init_node('Kalman_node', anonymous=True)
	r=rospy.Rate(10)
	obj=wp()
	obj.run()
	#Average absolute error
	print("Erro avA X: ",av_error[0]/count, "Erro avA Y: ",  av_error[1]/count, "Erro avA Z: ", av_error[2]/count)
	#Average relative error
	print("Erro avR X: ",avR_error[0]/count, "Erro avR Y: ",  avR_error[1]/count, "Erro avR Z: ", avR_error[2]/count)
	#Maximum absolute error
	print("Erro maxA X: ", max_error[0], "Erro maxA Y: ", max_error[1],"Erro maxA Z: ", max_error[2])
	
	obj.drawgraphic()
