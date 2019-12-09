function [out] = solving_cubic(x,y)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
abcd=[y,-x^2/2,0,-x^4/factorial(4)];
a=abcd(2)/abcd(1); b=abcd(3)/abcd(1); c=abcd(4)/abcd(1);

p=b-a^2/3;
q=c+2*a^3/27-b*a/3;

aux1=(-q/2+sqrt(q^2/4+p^3/27))^(1/3)+(-q/2-sqrt(q^2/4+p^3/27))^(1/3);
aux2=(-q/2+sqrt(q^2/4+p^3/27))^(1/3)*exp(1i*2*pi/3)+(-q/2-sqrt(q^2/4+p^3/27))^(1/3)*exp(1i*4*pi/3);
aux3=(-q/2+sqrt(q^2/4+p^3/27))^(1/3)*exp(1i*4*pi/3)+(-q/2-sqrt(q^2/4+p^3/27))^(1/3)*exp(1i*2*pi/3);

roots=[aux1, aux2, aux3]-ones(1, 3)*(a/3);

out=roots(1);
end

