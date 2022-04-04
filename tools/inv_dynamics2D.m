function [M, C, g]=inv_dynamics2D(tau, theta, q, dtheta, dq, hb)
syms M C tau1 tau2 tau3 tau4 x1 x2 x3 x4 dx1 dx2 dx3 dx4 m1 m2 m3 m4 I1 I2 I3 I4 l g0 mb Ib l w real

%gravity contribution
%Ub=constant
%forse le U sono invertite di segno (oppure vanno bene, ma asse y deve
%puntare verso il basso)
U1=m1*g0*[hb-l/2*sin(q1)]'; U2=m2*g0*[hb-l*sin(q1)-l/2*sin(q2)]';
U3=m3*g0*[hb-l/2*sin(q3)]'; U4=m4*g0*[hb-l*sin(q3)-l/2*sin(q4)]';
U=U1+U2+U3+U4;
g=simplify(jacobian(U,q)')

%inertia matrix
%Ib=w*l^3/12; %l>w
Mb=2*Ib;
pc2=[l*cos(q1)+0.5*l*cos(q2); l*sin(q1)+0.5*l*sin(q2)];
pc4=[l*cos(q3)+0.5*l*cos(q4); l*sin(q3)+0.5*l*sin(q4)];
vc2=vc_diff(pc2, q, dq);
vc4=vc_diff(pc4, q, dq);
simplify(vc2'*vc2)
T1=0.5*I1*dq1^2+0.5*m1*(l^2/4)*dq1^2;
T2=0.5*I2*(dq1+dq2)^2+0.5*m2*vc2'*vc2;
T3=0.5*I3*dq3^2+0.5*m3*(l^2/4)*dq3^2;
T4=0.5*I4*(dq3+dq4)^2+0.5*m4*vc4'*vc4;
T12=simplify(T1+T2);
T34=simplify(T3+T4);
M12=Inertia(T12, [q1 q2], [dq1 dq2])
M34=Inertia(T34, [q3 q4], [dq3 dq4])
Ml=[M12 zeros(2, 2);
    zeros(2,2) M34];
M=[Mb  zeros(1,4);
 zeros(4,1)  Ml];
C=fact_matrix(M, [theta q]', [dtheta dq]');
dM=dtot(M, [theta q]', [dtheta dq]');
skew=study_symm(dM-2*C, "skew")

end