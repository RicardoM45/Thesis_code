close all;
clear;

%o x1 e y1 fiz de cabeça e foram usados para relacionar o 'c' e o 'C', cuja
%relação é C=c.

x2=0.0001;
x=linspace(0,x2,102);
y2=2;

w=11.6/1.55*10^-3;

% Aproximação de Taylor/Maclaurin
a=1; b=-y2; c=x2^2;

c1=solving_cubic(x2,y2);

c_est2=(-b-sqrt(b^2-4*a*c))/(2*a);
c_est3=(-b+sqrt(b^2-4*a*c))/(2*a);
c_est=x2^2/(y2*2);

c=c1;
count=0;
if(abs(x2)>=0.001)
    erro=inf;
    while(erro>0.001)
        f=y2-(c*cosh(x2/c)-c); %Falar no rel que converge mas com o ^2 para se ter o zero ser o minimo, caso contrario esta mal
        df=x2/c*sinh(x2/c)-cosh(x2/c)+1;

        c=c-f/df;
        erro=abs(f);
        count=count+1;
    end

    s=c*sinh(x2/c);
    
    y=c*cosh(x/c)-c;
    y_est=c1*cosh(x/c_est)-c_est;
%     plot(x, y_est, x2,y2, 'X')
    
    plot(x, y, x2,y2, 'X', 'LineWidth', 3);
    title('Point at x=0.1 and y=2', 'Fontsize', 18);
    xlabel('radial distance/m', 'Fontsize', 16);
    ylabel('height/m', 'Fontsize', 16);
    legend("Catenary Curve");
else
    s=y2;
    plot(x2,y2, 'X', 'LineWidth', 3, 'Color', 'red');
    line([0 0], [0 y2], 'LineWidth', 3);
    title('Point at x=0.1 and y=2', 'Fontsize', 18);
    xlabel('radial distance/m', 'Fontsize', 16);
    ylabel('height/m', 'Fontsize', 16);
    legend("Catenary Curve");
    xlim([-0.5, 0.5]);
end

T_vert=w*s;
T0=w*c;

aa=x;
x2=c1;
aux1=abs((aa.*cosh(x2./aa).*x2.^2./aa.^2-aa).*x2.^2./aa.^3.*cosh(x2./aa));
aux2=abs((-cosh(x2./aa)+x2./aa.*sinh(x2./aa)).^2);
ratio=aux1./aux2;
% figure;
% plot(aa, ratio);
