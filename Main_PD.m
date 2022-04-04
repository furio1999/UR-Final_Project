% UR Project
% Borrelli - De Pucchio - Sanguigni
clear; close all; clc

addpath("tools");

% Definition of symbolic variables
syms theta q1 q2 q3 q4 dtheta dq1 dq2 dq3 dq4 hb real
syms x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 dx1 dx2 dx3 dx4 dx5 dx6 dx7 dx8 dx9 dx10 tau real

q=[q1 q2 q3 q4];                % Joint position
dq=[dq1 dq2 dq3 dq4];           % Joint velocities
% body
g0=9.81;                        %[m/s^2]
hb=1.5;                         % initial height [m]
h=0.1; b=0.4;                   % central body (rectangular) [m]
mb=0.5;                         % mass of central bopdy [kg]
Ib=mb*(h^2+b^2)/3;              % central body inertia
%dati giunti
m=0.05; m1=m; m2=m; m3=m; m4=m; % joint masses [kg]
l=0.03;                         % legs length [m]
I=m1*l^2/12; I1=I; I2=I; I3=I; I4=I; % legs intertia


%% PD + Euler Integration

tau=zeros(5,1);
Kp1=200; Kp2=50; Kp3=50; Kp4=50; Kp5=50;
Kd1=50;  Kd2=25;  Kd3=25;  Kd4=25; Kd5=25;
% trade-off: kd1 low-->x2 overshoots; kd1 high-->x1 converges slowly
x1_des=0; dx1_des=0; ddx1_des=0; x3_des=3*pi/4; dx3_des=0; ddx3_des=0;
x5_des=pi/4; dx5_des=0; ddx5_des=0; x7_des=pi/4; dx7_des=0; ddx7_des=0; 
x9_des=pi/4; dx9_des=0; ddx9_des=0;

disp("**Integration**");
h=0.02;
t=0:h:3; %l'obbiettivo del paper era stabilizzare theta entro 0.5 sec
x=zeros(10, length(t));
tau_tot=zeros(5, length(t));
x(:,1)=[pi/4 0 pi/2 0 pi/2 0 pi/2 0 pi/2 0]'; %Initial value

dpos=zeros(5,1);

for i=1:length(t)-1
    x1=x(1,i); x2=x(2,i); x3=x(3,i); x4=x(4,i); x5=x(5,i);
    x6=x(6,i); x7=x(7,i); x8=x(8,i); x9=x(9,i); x10=x(10,i);
    
    % dx=f(x,u) 
    dx1= x2;
    dx3= x4;
    dx5= x6;
    dx7= x8;
    dx9= x10;
    
    invMx=[  12/(mb*(b^2 + h^2)),                             -12/(mb*(b^2 + h^2)),           0,                                  -12/(mb*(b^2 + h^2)),           0;
        -12/(mb*(b^2 + h^2)),   (12*(mb*b^2 + mb*h^2 + m*l^2))/(l^2*m*mb*(b^2 + h^2)), -12/(l^2*m),                                   12/(mb*(b^2 + h^2)),           0;
        0,                                             -12/(l^2*m),  24/(l^2*m),                                                     0,           0;
        -12/(mb*(b^2 + h^2)),                                      12/(mb*(b^2 + h^2)),           0, (12*(mb*b^2 + mb*h^2 + m*l^2))/(l^2*m*mb*(b^2 + h^2)), -12/(l^2*m);
        0,                                                       0,           0,                                           -12/(l^2*m),  24/(l^2*m);];

    cx=zeros(5,1);

    gx=  -[ -(g0*m*(3*l*cos(x1 + x3) + l*cos(x1 + x5) + 3*l*cos(x1 + x7) + l*cos(x1 + x9) + 4*h*cos(x1)))/2;
        -(3*g0*l*m*cos(x1 + x3))/2;
        -(g0*l*m*cos(x1 + x5))/2;
        -(3*g0*l*m*cos(x1 + x7))/2;
        -(g0*l*m*cos(x1 + x9))/2; ];

    % Inertia Matrix
    Mx=inv(invMx);


    %I/O linearization
    v1=ddx1_des+Kp1*(x1_des-x1)+Kd1*(dx1_des-dx1);
    v2=ddx3_des+Kp2*(x3_des-x3)+Kd2*(dx3_des-dx3);
    v3=ddx5_des+Kp3*(x5_des-x5)+Kd3*(dx5_des-dx5);
    v4=ddx7_des+Kp4*(x7_des-x7)+Kd4*(dx7_des-dx7);
    v5=ddx9_des+Kp5*(x9_des-x9)+Kd5*(dx9_des-dx9); 
    %v3=0; v4=0; v5=0;
    v=[v1 v2 v3 v4 v5]';
    % case 1: We control theta q1 q2 q3
    temp=-pinv(Mx(1,5))*(Mx(1,1)*v1+cx(1)+gx(1));
    tau(2:5)=Mx(2:5, 5)*temp+Mx(2:5, 1:4)*v(1:4)+cx(2:5)+gx(2:5);
    % end case 1
    % case 2: We control only theta e q1 
%     temp= -pinv(Mx(1:2:5,3:5))*(Mx(1:2:5,1:2)*[v1 v2]'+cx(1:2:5)+gx(1:2:5)); %ddq2 ddq3 ddq4
%     tau(2)= Mx(2, :)*[v1 v2 0 0 0]' + Mx(2, :)*[0 0 temp']' + cx(2) + gx(2);
%     tau(4)= Mx(4, :)*[v1 v2 0 0 0]' + Mx(4, :)*[0 0 temp']' + cx(4) + gx(4);
    % Mx*[ddtheta ddq1 ... ddq4]' = Mx(:, 5)*ddq4 + Mx(:, 1:4)*[ddtheta ... ddq3];
    % end case 2
    

    tau_tot(:, i)=tau;
    dpos=invMx*(tau-gx-cx);
    
    %dx=f(x,u)
    dx2= dpos(1);
    dx4= dpos(2);
    dx6= dpos(3);
    dx8= dpos(4);
    dx10=dpos(5);
    % Euler Integration
    f=[dx1 dx2 dx3 dx4 dx5 dx6 dx7 dx8 dx9 dx10]';
    x(:,i+1)=x(:,i)+h*f;

    %tau(2)=-gx(2); tau(5)=-gx(5);
end


disp("**plot**");
%angolo espresso in gradi con rad2deg
figure()
plot(t,(x(1:2, :)),'LineWidth',1.25);
xlabel('Time [s]')
title('\theta and d\theta  behavior')
legend('\theta [Rad]', 'd\theta  [Rad/s]', 'best')
figure()
plot(t,(x(3:10, :)),'LineWidth', 1.25);
xlabel('Time [s]')
legend( 'q_1 [rad]', 'dq_1  [rad/s]','q2 [rad]', 'dq2 [rad/s]', 'q3 [rad]', ...
    'dq3 [rad/s]','q_4 [rad]', 'dq_4  [rad/s]', 'best')
title("Joint angular position and velocity behaviour")
figure()
plot(t, tau_tot,'LineWidth',1.25);
xlabel('Time [s]')
legend('\tau_1', '\tau_2', '\tau_3', '\tau_4', '\tau_5')
title("Control inputs")

%

