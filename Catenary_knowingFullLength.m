close all;
clear;

% Initializations
xf=2;
xi=1;
x=linspace(xi,xf,501);
if(abs(xf)<0.001)%Case where the tether's shape is approximately a vertical line
    xf=0.001;
end
s_tot=5;
deltaX=abs((xf-xi)/2);
yf=3;
yi=1;
deltaY=yf-yi;
x_av=(xf+xi)/2;
if(deltaY==0) %Sums a small offset
    deltaY=deltaY+0.001;
end

alfa=sinh(atanh(deltaY/s_tot)); %Auxiliar variable
% Results from the approximation to the Taylor serie 
a=deltaY/(2*alfa)-deltaX;
b=-deltaX^3/factorial(3);
c=-deltaX^5/factorial(5);
x1=(-b+sqrt(b^2-4*a*c))/(2*a);
x2=(-b-sqrt(b^2-4*a*c))/(2*a);
c1=sqrt(x1); 
c2=-sqrt(x1);
c3=sqrt(x2);
c4=-sqrt(x2);

%Iterative Newton-Raphson method to compute parameter <a>
a=c1;
erro=inf;
count=0;
A=atanh(deltaY/s_tot); %Auxiliar variable
while(erro>0.001)
    f=deltaY-2*a*sinh(atanh(deltaY/s_tot))*sinh(deltaX/a);
    df=2*sinh(A)*(deltaX/a*cosh(deltaX/a)-sinh(deltaX/a));
    a=a-f/df;
    erro=abs(f);
    count=count+1;
end
x0=x_av-a*atanh(deltaY/s_tot);

%Computing the parameter <C>
yy=a.*cosh((x-x0)./a);
aux=a*cosh((xf-x0)/a);
C=((yf-aux)+(yi-yy(1)))/2;
yy=a*cosh((x-x0)/a)+C;

%Plots
plot(x, yy, xi, yi, 'x', xf, yf, 'x', 'Linewidth', 3);
if(abs(xf)<0.001)
    xlim([-0.5,0.5]);
end