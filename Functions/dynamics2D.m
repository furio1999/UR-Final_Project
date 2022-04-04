function [M, c, g]=dynamics2D(theta, q, dtheta, dq)

syms g0 M C m I m1 m2 m3 m4 I1 I2 I3 I4 l g0 mb Ib l w h b hb real
% comment this part if you want numeric results
%{

% Values
g0=9.81
hb=1.5; %Height of the body from the terrain [m]
h=0.1; b=0.4; % Body dimensions [m]
mb=0.5; % Body mass [kg]

m=0.05; 
l=0.03; % legs length [m]
Ib=mb*(h^2+b^2)/12; % Intertia of central body
m1=m; m2=m; m3=m; m4=m; % legs masses [kg]
I=m*l^2/12; 
I1=I; I2=I; I3=I; I4=I; % Legs Inertia

%} 
% end comment
%% 
q1=q(1); q2=q(2); q3=q(3); q4=q(4); dq1=dq(1); dq2=dq(2); dq3=dq(3); dq4=dq(4);

% Legs CoMs positions wrt the world frame
pc1=[0.5*l*cos(q1+theta)-0.5*b*cos(theta); hb-0.5*h*sin(theta)-0.5*l*sin(q1+theta)];
pc2=[l*cos(q1+theta)+0.5*l*cos(q2+theta)-0.5*b*cos(theta); hb-0.5*h*sin(theta)-l*sin(q1+theta)-0.5*l*sin(q2+theta)];
pc3=[0.5*l*cos(q1+theta)+0.5*b*cos(theta); hb-0.5*h*sin(theta)-0.5*l*sin(q1+theta)];
pc4=[l*cos(q3+theta)+0.5*l*cos(q4+theta)+0.5*b*cos(theta); hb-0.5*h*sin(theta)-l*sin(q3+theta)-0.5*l*sin(q4+theta)]; 
% var= [theta q];
% dvar= [dtheta dq];
% vc=dpc/dt=dpc/dq* dq/dt
vc1=vc_diff(pc1,[theta q],[dtheta dq]);
vc2=vc_diff(pc2, [theta q], [dtheta dq]);
vc3=vc_diff(pc3, [theta q], [dtheta dq]);
vc4=vc_diff(pc4, [theta q], [dtheta dq]);

% kinetic energy
% to simplify the computation: use only rotational kinetic energy
Tb=0.5*Ib*dtheta^2; % body
T1=0.5*I1*(dq1+dtheta)^2+0.5*m1*vc1'*vc1;        % leg 1 
T2=0.5*I2*(dq1+dq2+dtheta)^2+0.5*m2*vc2'*vc2;   % leg 2
T3=0.5*I3*(dq3+dtheta)^2+0.5*m3*vc3'*vc3;        % leg 3
T4=0.5*I4*(dq3+dq4+dtheta)^2+0.5*m4*vc4'*vc4;   % leg 4

% alternative way for the computation of M
% T12=simplify(T1+T2);
% T34=simplify(T3+T4);
% Inertia Matrix
% legs
% M12= Inertia(T12, [theta q1 q2], [dtheta dq1 dq2]); %T=q'*M*q;
% M34= Inertia(T34, [theta q3 q4], [dtheta dq3 dq4]);
% Ml=[M12 zeros(2, 2);
%     zeros(2,2) M34];
% Total Inertia
% M=[Mb  zeros(1,4);
%  zeros(4,1)  Ml];
T= Tb+T1+T2+T3+T4; % total Kinetic energy
M= Inertia(T, [theta q], [dtheta dq]);

c=Coriolis(M, [ theta q]', [dtheta dq]'); % Coriolis Term
C=fact_matrix(M, [theta q]', [dtheta dq]');  % Coriolis Matrix
% check for the condition dM-2S-->skew symmetric
% dM= dtot(M, [theta q]', [dtheta dq]'); %dM/dt
% skew = study_symm(dM-2*C, "skew");

% Potential Energy. We are not interested in the moments AFTER the impact
% on the ground, so the body part is irrelevant here
U1=m1*g0*[hb-l/2*sin(q1+theta)-0.5*h*sin(theta)]';                   % Leg 1
U2=m2*g0*[hb-l*sin(q1+theta)-l/2*sin(q2+theta)-0.5*h*sin(theta)]';   % Leg 2
U3=m3*g0*[hb-l/2*sin(q3+theta)-0.5*h*sin(theta)]';                   % Leg 3
U4=m4*g0*[hb-l*sin(q3+theta)-l/2*sin(q4+theta)-0.5*h*sin(theta)]';   % Leg 4
U=U1+U2+U3+U4;
% Derive g term
g=simplify(jacobian(U,[theta q])');
end