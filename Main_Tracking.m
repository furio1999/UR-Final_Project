clear; close all; clc

Sdes= load('ref_trajectory.mat');
des= Sdes.xHistory';
% data
g0=9.81;
mb=0.5;
m=0.05;
l=0.03;
h=0.1; 
b=0.4;

% Inertia Matrix & its inverse
Mx=[ (mb*b^2)/12 + (mb*h^2)/12 + (m*l^2)/3,  (l^2*m)/6, (l^2*m)/12,  (l^2*m)/6, (l^2*m)/12;
                             (l^2*m)/6,  (l^2*m)/6, (l^2*m)/12,          0,          0;
                            (l^2*m)/12, (l^2*m)/12, (l^2*m)/12,          0,          0;
                             (l^2*m)/6,          0,          0,  (l^2*m)/6, (l^2*m)/12;
                            (l^2*m)/12,          0,          0, (l^2*m)/12, (l^2*m)/12];
invMx=[  12/(mb*(b^2 + h^2)), -12/(mb*(b^2 + h^2)), 0, -12/(mb*(b^2 + h^2)), 0;
        -12/(mb*(b^2 + h^2)),   (12*(mb*b^2 + mb*h^2 + m*l^2))/(l^2*m*mb*(b^2 + h^2)), -12/(l^2*m),                                   12/(mb*(b^2 + h^2)),           0;
        0, -12/(l^2*m),  24/(l^2*m), 0, 0;
        -12/(mb*(b^2 + h^2)), 12/(mb*(b^2 + h^2)), 0, (12*(mb*b^2 + mb*h^2 + m*l^2))/(l^2*m*mb*(b^2 + h^2)), -12/(l^2*m);
        0, 0, 0, -12/(l^2*m), 24/(l^2*m);];
% Inizianlization terms
Ts=0.005;
t=(0:Ts:3)';
x=des; %zeros(size(des));
Kp1=200; Kpj=50;
Kd1=100;  Kdj=50;  

% PD with Euler Integration
for i=1:length(t)-1
    gx=  -[ -(g0*m*(3*l*cos(x(i,1) + x(i,3)) + l*cos(x(i,1) + x(i,5)) + 3*l*cos(x(i,1) + x(i,7)) + l*cos(x(i,1) + x(i,9)) + 4*h*cos(x(i,1))))/2;
        -(3*g0*l*m*cos(x(i,1) + x(i,3)))/2;
        -(g0*l*m*cos(x(i,1) + x(i,5)))/2;
        -(3*g0*l*m*cos(x(i,1) + x(i,7)))/2;
        -(g0*l*m*cos(x(i,1) + x(i,9)))/2; ];

    dx1= x(i,2);
    dx3= x(i,4);
    dx5= x(i,6);
    dx7= x(i,8);
    dx9= x(i,10);

    %I/O linearization
    v1=Kp1*(des(i,1)-x(i,1))+Kd1*(des(i,2)-dx1);
    v2=Kpj*(des(i,3)-x(i,3))+Kdj*(des(i,4)-dx3);
    v3=Kpj*(des(i,5)-x(i,5))+Kdj*(des(i,6)-dx5);
    v4=Kpj*(des(i,7)-x(i,7))+Kdj*(des(i,8)-dx7);
    v5=Kpj*(des(i,9)-x(i,9))+Kdj*(des(i,10)-dx9); 

    v=[v1 v2 v3 v4 v5]';
    % alternative formulation:
    % temp=-pinv(Mx(1,5))*(Mx(1,1)*v1+cx(1)+gx(1));
    % tau(2:5)=Mx(2:5, 5)*temp+Mx(2:5, 1:4)*v(1:4)+gx(2:5);
    u= Mx+cx+gx;

    %tau_tot(:, i)=tau;
    dpos=invMx*(u-gx);
    dx2= dpos(1);
    dx4= dpos(2);
    dx6= dpos(3);
    dx8= dpos(4);
    dx10=dpos(5);
    f=[dx1 dx2 dx3 dx4 dx5 dx6 dx7 dx8 dx9 dx10];
    % Euler integration
    x(i+1,:)=x(i,:)+Ts*f;
end

figure()
plot(t,rad2deg((x(:,1))),'LineWidth',1.25);
hold on
plot(t,rad2deg((des(:,1))),'LineWidth',1.25);
xlabel('Time [s]')
title('\theta and \theta_{ref}  behavior')
legend('\theta [deg]', '\theta_{ref}  [deg/s]', 'best')

figure()
subplot(2,2,1)
plot(t,rad2deg((x(:,3))),'LineWidth',1.25);
hold on
plot(t,rad2deg((des(:,3))),'LineWidth',1.25);
xlabel('Time [s]')
title('q1 and q1_{ref}  behavior')
legend('q1 [deg]', 'q1_{ref}  [deg/s]', 'Location','southeast')

subplot(2,2,2)
plot(t,rad2deg((x(:,5))),'LineWidth',1.25);
hold on
plot(t,rad2deg((des(:,5))),'LineWidth',1.25);
xlabel('Time [s]')
title('q2 and q2_{ref}  behavior')
legend('q2 [deg]', 'q2_{ref}  [ded/s]', 'Location','southeast')

subplot(2,2,3)
plot(t,rad2deg((x(:,7))),'LineWidth',1.25);
hold on
plot(t,rad2deg((des(:,7))),'LineWidth',1.25);
xlabel('Time [s]')
title('q3 and q3_{ref}  behavior')
legend('q3 [deg]', 'q3_{ref}  [deg/s]', 'Location','southeast')

subplot(2,2,4)
plot(t,rad2deg((x(:,9))),'LineWidth',1.25);
hold on
plot(t,rad2deg((des(:,9))),'LineWidth',1.25);
xlabel('Time [s]')
title('q4and q4_{ref}  behavior')
legend('q4 [deg]', 'q4_{ref}  [deg/s]', 'Location','southeast')