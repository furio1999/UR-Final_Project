function  dx = model01(x, tau)
% Continuos model to discretize
g0=9.81;                        %[m/s^2]
hb=1.5;                         % altezza da terra [m]
h=0.1; b=0.4;                   % dimensioni corpo centrale [m]
mb=0.5;                         % massa corpocentrale [kg]
Ib=mb*(h^2+b^2)/3;              % inerzia piastra modellata piana
%dati giunti
m=0.05; m1=m; m2=m; m3=m; m4=m; % massa dei giunti [kg]
l=0.03;                         % lunghezza dei giunti [m]
I=m1*l^2/12; I1=I; I2=I; I3=I; I4=I; % inerzia giunti, modellati come aste, associato al proprio centro di massa.
% Inverse of Inverse Matrix
invMx=[  12/(mb*(b^2 + h^2)),                             -12/(mb*(b^2 + h^2)),           0,                                  -12/(mb*(b^2 + h^2)),           0;
    -12/(mb*(b^2 + h^2)),   (12*(mb*b^2 + mb*h^2 + m*l^2))/(l^2*m*mb*(b^2 + h^2)), -12/(l^2*m),                                   12/(mb*(b^2 + h^2)),           0;
    0,                                             -12/(l^2*m),  24/(l^2*m),                                                     0,           0;
    -12/(mb*(b^2 + h^2)),                                      12/(mb*(b^2 + h^2)),           0, (12*(mb*b^2 + mb*h^2 + m*l^2))/(l^2*m*mb*(b^2 + h^2)), -12/(l^2*m);
    0,                                                       0,           0,                                           -12/(l^2*m),  24/(l^2*m);];
% Coriolis term vector
cx=zeros(5,1);
% Potential term vector
gx=  [ -(g0*m*(3*l*cos(x(1) + x(3)) + l*cos(x(1) + x(5)) + 3*l*cos(x(1) + x(7)) + l*cos(x(1) + x(9)) + 4*h*cos(x(1))))/2;
    -(3*g0*l*m*cos(x(1) + x(3)))/2;
    -(g0*l*m*cos(x(1) + x(5)))/2;
    -(3*g0*l*m*cos(x(1) + x(7)))/2;
    -(g0*l*m*cos(x(1) + x(9)))/2; ];

% Inertia matrix
% Mx=[ (mb*b^2)/12 + (mb*h^2)/12 + (m*l^2)/3,  (l^2*m)/6, (l^2*m)/12,  (l^2*m)/6, (l^2*m)/12;
%                              (l^2*m)/6,  (l^2*m)/6, (l^2*m)/12,          0,          0;
%                             (l^2*m)/12, (l^2*m)/12, (l^2*m)/12,          0,          0;
%                              (l^2*m)/6,          0,          0,  (l^2*m)/6, (l^2*m)/12;
%                             (l^2*m)/12,          0,          0, (l^2*m)/12, (l^2*m)/12]

% Kinematic model
dx(1)= x(2);
dx(3)= x(4);
dx(5)= x(6);
dx(7)= x(8);
dx(9)= x(10);

dpos=invMx*(tau-gx-cx);  % ?this should be 0, so we reach steady state?
dx(2)= dpos(1);
dx(4)= dpos(2);
dx(6)= dpos(3);
dx(8)= dpos(4);
dx(10)=dpos(5);

dx=dx';

end