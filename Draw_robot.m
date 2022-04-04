clc; close all; clear;
addpath("Datasets");

% v = VideoWriter('animation_MPC_new2', "MPEG-4");
% v.FrameRate = 5; %default 3.5, parziale 1
% open(v)


% dimensions of the body
H=0.1; L=0.4; l=0.1;
rgb="r";

% full dataset animation
% S=load("xcompleto_1000_filtered");
% data=S.xcompleto2;
% dataset= data{1};
% for i= 2:length(data)
%     % extract the data
%     tmp= data{i};
%     % concatenate
%     dataset = cat(1,dataset,tmp);
% end

% neural network animation
% S=load("theta_40");
% alfa=S.y;
% S1=load("q1_40");
% alfa1=S1.y;
% S2=load("q2_40");
% alfa2=S2.y;
% S3=load("q1_40");
% alfa3=S3.y;
% S4=load("q2_40");
% alfa4=S4.y;

S=load("History5.mat");
dataset= S.xHistory;
alfa = dataset(1,:); % target trajectory (theta in the report)
alfa1=dataset(3,:); %q1
alfa2=dataset(5,:); %q2
alfa3=dataset(7,:); %q3
alfa4=dataset(9,:); %q4

dt=0.1;
g=9.81;
center_location=[0,1.8]; %start position
center1=center_location(1); %x-coordinate
center2=center_location(2)-0.5*g*dt^2; %y-coordinate

for i=1:length(alfa) %length(alfa)*dt;
center2=center2-0.5*g*dt^2; %traslation
theta=alfa(i);
q1=alfa1(i);
q2=alfa2(i);
q3=alfa3(i);
q4=alfa4(i);
R= ([cos(theta), -sin(theta); sin(theta), cos(theta)]);
X=([-L/2, L/2, L/2, -L/2]); % X expressed in robot frame
Y=([-H/2, -H/2, H/2, H/2]);

for k=1:4
T(:,k)=R*[X(k); Y(k)]; % compute rotation
end
% apply rotation
x_lower_left=center1+T(1,1);
x_lower_right=center1+T(1,2);
x_upper_right=center1+T(1,3);
x_upper_left=center1+T(1,4);
y_lower_left=center2+T(2,1);
y_lower_right=center2+T(2,2);
y_upper_right=center2+T(2,3);
y_upper_left=center2+T(2,4);

% body (expressed in world frame)
x_coor=[x_lower_left x_lower_right x_upper_right x_upper_left]; 
y_coor=[y_lower_left y_lower_right y_upper_right y_upper_left];
a=patch('Vertices',[x_coor; y_coor]','Faces',[1 2 3 4],'Edgecolor',rgb,'Facecolor','none','Linewidth',1.2);

% legs coordinates (robot frame)
X1=([-L/2, -L/2-l*cos(q1)]); 
Y1=([-H/2, -H/2-l*sin(q1)]);

X2=([-L/2-l*cos(q1), -L/2-l*cos(q1)-l*cos(q2)]); 
Y2=([-H/2-l*sin(q1), -H/2-l*sin(q1)-l*sin(q2)]);

X3=([L/2, L/2-l*cos(q3)]); 
Y3=([-H/2, -H/2-l*sin(q3)]);

X4=([L/2-l*cos(q3), L/2-l*cos(q3)-l*cos(q4)]); 
Y4=([-H/2-l*sin(q3), -H/2-l*sin(q3)-l*sin(q4)]);

% rotations
for j=1:2
T1(:,j)=R*[X1(j); Y1(j)]; 
T2(:,j)=R*[X2(j); Y2(j)]; 
T3(:,j)=R*[X3(j); Y3(j)]; 
T4(:,j)=R*[X4(j); Y4(j)]; 
end

%apply rotations to the legs (2 sublegs for each leg)
xgambetta1(1)=center1+T1(1,1);
xgambetta1(2)=center1+T1(1,2);
ygambetta1(1)=center2+T1(2,1);
ygambetta1(2)=center2+T1(2,2);

xgambetta2(1)=center1+T2(1,1); 
xgambetta2(2)=center1+T2(1,2);
ygambetta2(1)=center2+T2(2,1);
ygambetta2(2)=center2+T2(2,2);

xgambetta3(1)=center1+T3(1,1); 
xgambetta3(2)=center1+T3(1,2);
ygambetta3(1)=center2+T3(2,1);
ygambetta3(2)=center2+T3(2,2);

xgambetta4(1)=center1+T4(1,1); 
xgambetta4(2)=center1+T4(1,2);
ygambetta4(1)=center2+T4(2,1);
ygambetta4(2)=center2+T4(2,2);

% drawing the robot
b=patch('Vertices',[xgambetta1; ygambetta1]','Faces',[1 2],'Edgecolor',rgb,'Facecolor','none','Linewidth',1.2);
c=patch('Vertices',[xgambetta2; ygambetta2]','Faces',[1 2],'Edgecolor',rgb,'Facecolor','none','Linewidth',1.2);
d=patch('Vertices',[xgambetta3; ygambetta3]','Faces',[1 2],'Edgecolor',rgb,'Facecolor','none','Linewidth',1.2);
e=patch('Vertices',[xgambetta4; ygambetta4]','Faces',[1 2],'Edgecolor',rgb,'Facecolor','none','Linewidth',1.2);
axis equal

% set the limits in drawing coordinates
xlim([-0.4 0.4]);
ylim([0 2]); %4 o 3.5 upper limit
drawnow;
pause(dt);

% write the video
% frame = getframe(gcf);
% writeVideo(v,frame);

% this avoid superposition of drawings in subsequents frames
if i < length(alfa)
delete(a);
delete(b);
delete(c);
delete(d);
delete(e);
end
if ygambetta3(2)<0.1
    break;
end
end

pause
%close(v);
