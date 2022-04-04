clear; clc; close all;
addpath("datasets");

% start comment section here if necessary using "{" after the "%" sign
S= load('prova_400_matrici.mat');
data=S.xcompleto;
dataset= data{1};
for i= 2:length(data)
    % estraiamo la matrice da data
    tmp= data{i};
    % concateniamo
    dataset = cat(1,dataset,tmp);
end
theta = dataset(1:10:end,:); % traiettoria, target
theta0= theta(:,1); %theta iniziale, input
inputs=theta0';
targets=theta';
% Feedforward Neural Networks 
%net = feedforwardnet([512 512],'traingdm'); %original neural network proposed
net = feedforwardnet([50 100 512],'trainrp');
%net.divideFcn = ''; %default dividerand 60% train 20% validation 20% test
net.trainParam.epochs=300; %default 1000
net.trainParam.goal = 1e-5; %0 default
net.performParam.regularization = 0.6; %0 default %0.6 ottimo con 40 samples
[net,tr] = train(net,inputs,targets);
%}

%use this 2 commands below to reproduce our results on theta (comment the previous lines)
% S=load("rete_neurale_400.mat");
% net=net.net;
% tr=S.tr;
val0=deg2rad(60);
y = net(val0); %only values between 30 and 155 degrees

figure()
perf=plotperf(tr)
plot(0:0.01:6,rad2deg(y'),'LineWidth',1.25)
ylabel('theta [degree]')
xlabel('Time [s]')

