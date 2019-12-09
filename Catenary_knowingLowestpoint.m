close all;
clear;

%Initialization of the quadcopter's point (x2, y2)
x2=1;
y2=3;
x=linspace(0,x2,102);

% Results coming from the approximation to the Taylor/Maclaurin serie and
% using the Cardano's formula do compute the parameter <a> (solving_cubic
% fucntion)
a_est=solving_cubic(x2,y2);

%Iterative Newton-Raphson method to compute parameter <a>
a=a_est;
count=0;
if(abs(x2)>=0.001)
    erro=inf;
    while(erro>0.001)
        f=y2-(a*cosh(x2/a)-a);
        df=x2/a*sinh(x2/a)-cosh(x2/a)+1;
        a=a-f/df;
        erro=abs(f);
        count=count+1;
    end
    
    %Catenary curve model using the parameter <a> initial estimate
    y_est=a_est*cosh(x/a_est)-a_est;
    %Catenary curve model using the parameter <a> from the Newton-Raphson method    
    y=a*cosh(x/a)-a;
    
    plot(x, y, x2,y2, 'X', 'LineWidth', 3);
    title('Point at x=0.1 and y=2', 'Fontsize', 18);
    xlabel('radial distance/m', 'Fontsize', 16);
    ylabel('height/m', 'Fontsize', 16);
    legend("Catenary Curve");
else %Case where the tether's shape is approximately a vertical line
    s=y2;
    plot(x2,y2, 'X', 'LineWidth', 3, 'Color', 'red');
    line([0 0], [0 y2], 'LineWidth', 3);
    title('Point at x=0.1 and y=2', 'Fontsize', 18);
    xlabel('radial distance/m', 'Fontsize', 16);
    ylabel('height/m', 'Fontsize', 16);
    legend("Catenary Curve");
    xlim([-0.5, 0.5]);
end