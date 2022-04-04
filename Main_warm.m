% UR Project
% Borrelli - De Pucchio - Sanguigni
clear;
close all;
clc

addpath("tools");
%% Extractin files
St= load('theta_40.mat');
theta0=St.y;
Sq1= load('q1_40.mat');
q10= Sq1.y;
Sq2= load('q2_40.mat');
q20= Sq2.y;
Sq3= load('q3_40.mat');
q30= Sq3.y;
Sq4= load('q4_40.mat');
q40= Sq4.y;

Sp= load('prova_40.mat');
Xp0=Sp.xcompleto{3};

%% Nonlinear MPC

% Initialization of MPC
nx = 10;    % number of states
ny = 5;     % number of outputs (in our case 4 joint angels + theta)
nu = 5;     % number of inputs that correspon to the number of manipulated variables
nlobj = nlmpc(nx,ny,nu);

% Specify the sample time and horizons of the controller.
Ts = 0.005; % sample time
nlobj.Ts = Ts;
nlobj.PredictionHorizon = 20;
nlobj.ControlHorizon = 10;

%Specify the state function for the controller and 
nlobj.Model.StateFcn = "model01_DT";
nlobj.Model.IsContinuousTime = false;

%number of optional parameters
nlobj.Model.NumberOfParameters = 1;

% Output function. Validate the prediction model functions.
nlobj.Model.OutputFcn = @(x,u,Ts) [x(1); x(3); x(5); x(7); x(9)];

% Initial values. 
x0 = [pi/2 0 pi/4 0 pi/4 0 pi/4 0 pi/4 0]';
u0 = zeros(5,1);
validateFcns(nlobj, x0, u0, [], {Ts});

% %CONSTRAINT. 
% nlobj.Weights.OutputVariables = [3 3];
% Constraint on input variation
nlobj.Weights.ManipulatedVariablesRate = ones(1,5)*0.1;
% nlobj.OV(1).Min = -10;
% nlobj.OV(1).Max = 10;
nlobj.MV(2).Min = 0;      % Minimum torque given [Nm]
nlobj.MV(3).Min = 0;    
nlobj.MV(4).Min = 0;
nlobj.MV(5).Min = 0;        
nlobj.MV(2).Max = 6;       % Maximum torque given [Nm]
nlobj.MV(3).Max = 6;
nlobj.MV(4).Max = 6;
nlobj.MV(5).Max = 6;
% extended kalman filter, for estimating plant states. 
% EKF = extendedKalmanFilter(@model01_DT,@model01_measfunc,[pi/2 0 pi/4 0 pi/4 0 pi/4 0 pi/4 0]');
% Define initial conditions for the simulation, initialize the extended
% Kalman filter state, and specify a zero initial manipulated variable value.
x = x0;
y = [x(1) x(3) x(5) x(7) x(9)];
mv = zeros(5,1);

% Specify the output reference value.
yref = [0 pi/2 pi/2 pi/2 pi/2];

%Create an nlmpcmoveopt object, and specify the sample time parameter.
nloptions = nlmpcmoveopt;
nloptions.Parameters = {Ts};
% Solver Constraints
nlobj.Optimization.SolverOptions.Algorithm = 'sqp';
nlobj.Optimization.SolverOptions.MaxIterations = 1300;
nlobj.Optimization.SolverOptions.MaxFunctionEvaluations = 1500;
nlobj.Optimization.SolverOptions.FiniteDifferenceType = 'central';
nlobj.Optimization.SolverOptions.OptimalityTolerance = 1e-3;

%% Warm Start
nloptions.X0=[theta0 Xp0(2,:)' q10 Xp0(4,:)' q20 Xp0(6,:)' q30 Xp0(8,:)' q40 Xp0(10,:)'];
% nloptions.X0=Xp0';
Duration =3;
xHistory = x;
tic
for ct = 1:(Duration/Ts)
    % Compute optimal control moves
    [mv,nloptions] = nlmpcmove(nlobj,x,mv,yref,[],nloptions);
    % Implement first optimal control move
    x=x+Ts*model01(x, mv);
    % Generate sensor data
    y = x([1 3 5 7 9]);
    % Save plant states
    xHistory = [xHistory x];
end
toc
%% PLOTS
% Ts = 0.1; % sampling time
% Duration = 1;

figure()
plot(0:Ts:Duration,rad2deg(xHistory(1,:)),"LineWidth",1.25)
xlabel('Time [s]')
ylabel('Theta [degree]')
title("Theta Behaviour");

figure()
subplot(2,2,1)
plot(0:Ts:Duration,rad2deg(xHistory(3,:)),"LineWidth",1.25)
xlabel('Time [s]')
ylabel('q1[degree]')
title("q1 Behaviour");

subplot(2,2,2)
plot(0:Ts:Duration,rad2deg(xHistory(5,:)),"LineWidth",1.25)
xlabel('Time [s]')
ylabel('q2[degree]')
title("q2 Behaviour");

subplot(2,2,3)
plot(0:Ts:Duration,rad2deg(xHistory(7,:)),"LineWidth",1.25)
xlabel('Time [s]')
ylabel('q3[degree]')
title("q3 Behaviour");

subplot(2,2,4)
plot(0:Ts:Duration,rad2deg(xHistory(9,:)),"LineWidth",1.25)
xlabel('Time [s]')
ylabel('q4[degree]')
title("q4 Behaviour");

