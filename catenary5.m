close all;
clear;

% Inicializa??es
xf=1.5;
aux1=abs(xf);
if(aux1<0.001)
    xf=0.001;
end
x=linspace(0,xf,501);
c_i=7;
x0=0.5;
const=0;
offset=0.001;

% Equa??o da caten?ria
y=c_i*cosh((x-x0)/c_i)+const;

% Equa??o do comprimento do fio
% s=c_i*sinh((x-x0)/c_i);
% % s_tot=s(501)-s(1);
% % Inicializa??es
% deltaX=(x(501)-x(1))/2;
% x_av=(x(501)+x(1))/2;
% deltaY=y(501)-y(1);

%%%%%%%%%%%%
s_tot=2;
deltaX=abs(xf/2);
yf=1;
yi=0.3;
deltaY=yf-yi;
x_av=xf/2;
if(deltaY==0)
    deltaY=deltaY+offset;
end
%%%%%%%%%%%%%
alfa=sinh(atanh(deltaY/s_tot));

% Aproxima??o de Taylor/Maclaurin ordem 3
c_f=sqrt((2*alfa*deltaX^3)/(factorial(3)*(deltaY-2*alfa*deltaX)));
c_f_est=c_f;

% Aproxima??o de Taylor/Maclaurin ordem 5
a=deltaY/(2*alfa)-deltaX;
b=-deltaX^3/factorial(3);
c=-deltaX^5/factorial(5);

x1=(-b+sqrt(b^2-4*a*c))/(2*a);
x2=(-b-sqrt(b^2-4*a*c))/(2*a);
c1=sqrt(x1); 
c2=-sqrt(x1);
c3=sqrt(x2);
c4=-sqrt(x2);

c_f=c1;
% M?todo de Newton-Raphson para calcular o 'c'
f=inf;
i=0;
tic
A=atanh(deltaY/s_tot);
while(abs(f)>0.000000000000001)
    f=deltaY-2*c_f*sinh(atanh(deltaY/s_tot))*sinh(deltaX/c_f);
    df=2*sinh(A)*(deltaX/c_f*cosh(deltaX/c_f)-sinh(deltaX/c_f));
%     df=-2*sinh(A)*exp(deltaX/c_f);
    c_f=c_f-f/df;
    i=i+1;
end
toc
x0_f=x_av-c_f*atanh(deltaY/s_tot);
% Plots dos gr?ficos
yy=c_f.*cosh((x-x0_f)./c_f);
aux=c_f*cosh((xf-x0_f)/c_f);
const_f=((yf-aux)+(yi-yy(1)))/2;
yy=c_f*cosh((x-x0_f)/c_f)+const_f;
plot(x, yy, 0, yi, 'x', xf, yf, 'x', 'Linewidth', 3);
s2=c_f*sinh((xf-x0_f)/c_f);
if(aux1<0.001)
    xlim([-0.5,0.5]);
end






% c_f=linspace(1,5,102);
% f=deltaY-2*c_f.*sinh(atanh(deltaY/s_tot)).*sinh(deltaX./c_f);
% f2=deltaY-2*c_f.*sinh(atan(deltaY/s_tot)).*sinh(deltaX./c_f);
% 
% df=-2*sinh(-atan(deltaY/s_tot)).*exp(deltaX./c_f);

% plot(c_f, f, c_f, f2);
% legend('tanh', 'tan');
% plot(x,y,x,s);
% legend('Normal', 'Comprimento');