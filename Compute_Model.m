clc; clear; close all
addpath("tools");
addpath("Functions");
addpath("Datasets");

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


%% Compute M, c and g
% comment this part if you want to use precomputed Mx, cx, gx
%{
disp("**Computation of Dynamics**");
% compute the dynamics using theta and q
[M, c, g] = dynamics2D(theta, q, dtheta, dq);
% if you want to load precomputed results
% filename="dynamics";
% vars={"M", "c", "g"};
% S=load("dynamics", vars{:});
% M=S.M; c=S.c; g=S.g;

% compute the dynamics in terms of x
% x1=theta, x2=dtheta, x3=q1, x4=dq1 ... x9=q4, x10=dq4
old=[theta dtheta q1 dq1 q2 dq2 q3 dq3 q4 dq4];
new=[x1 x2 x3 x4 x5 x6 x7 x8 x9 x10];
Mx=subs_reference(M, old, new); 
cx=subs_reference(c, old, new); 
gx=subs_reference(g, old, new);
%}
% end comment
%% Dynamic Model f(x,u)
disp("**dynamic model**");
dx1= x2;
dx3= x4;
dx5= x6;
dx7= x8;
dx9= x10;
% inv_Mx=inv(Mx) % compute from scratch: careful, time consuming
S=load("Matrices.mat"); Mx=S.Mx; cx=S.cx; gx=S.gx;
inv_Mx=S.inv_Mx;
dpos= inv_Mx*(tau - gx -cx);
dx2= dpos(1);
dx4= dpos(2);
dx6= dpos(3);
dx8= dpos(4);
dx10=dpos(5);
%% Equilibrium xeq
% to solve a nonloinear system in matlab, use x=solve([equations],
% [variable = new]). To find equilibrium positions, use Axeq+Bu=0.
eqns= [dx1 dx2 dx3 dx4 dx5 dx6 dx7 dx8 dx9 dx10]; 
disp("*operating point*")
op=load("xeq", "op");
op=op.op;
%% Linearization
disp("**Euler Equilibrium**");
% As=jacobian(eqns', new'); % Symblolic form of A, A=df(x,u)/dx
% Bs=jacobian(eqns', tau); % Symbolic form of B 
vars={"As", "Bs"};
S=load("Linear_Dynamics", vars{:}); As=S.As; Bs=S.Bs;

% now we find the numeric values at the equilibrium point
new = [pi/2 0 pi/4 0 pi/4 0 pi/4 0 pi/4 0 pi/4 0]';
gnum=subs_reference(gx, new, op);
tau_eq=sym(zeros(5,1)); 
% tau_eq(2)=0;
% tau_eq(5)=0;

% Linearization of the dynamics
A=subs_reference(As, [new', tau'], [op, tau_eq']);
B=subs_reference(Bs, [new', tau'], [op, tau_eq']);
%}