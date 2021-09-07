
%% SCRIPT_DrawCircle
% This script demonstrates use of the URsim class capabilities with the UR
% class. The URsim object is used to visualize the robot and calculate
% inverse kinematics for the system. Waypoints are executed by getting
% joint angle from the simulation object and sending them to the
% URToolboxScript controller running on the UR control box.
%
%   M. Kutzer, 14Mar2018, USNA
clearvars

% clearvars -EXCEPT hwObj
close all
clc

%% Create hardware flag (for debugging)
% -> Set value to true if you are connected to hardware
useHardware = false;
%% Initialize simulation
% -> This code has been tested on the UR5 and UR10 systems. Minor
% adjustments may be required to use it with the UR3.

if ~exist('simObj')
    % Create object
    simObj = URsim;
    % Initialize simulation
    simObj.Initialize;
    % Set a tool frame offset (e.g. for Robotiq end-effector not shown in
    % visualization)
%     simObj.FrameT = Tz(160);
     simObj.FrameT = Tz(0);
    
    % Hide frames
    frames = '0123456E';
    for i = 1:numel(frames)
        hideTriad(simObj.(sprintf('hFrame%s',frames(i))));
    end
end

%% Connect to hardware
% -> The message to the user *assumes* that you have:
%       (1) Properly installed URToolboxScript on the UR controller. See
%       the link below for instructions:
%
%       https://www.usna.edu/Users/weapsys/kutzer/_Code-Development/UR_Toolbox.php
%
%       (2) Configure the network card connecting the PC to the UR
%       controller to a static IP of 10.1.1.5
%
%       (3) Set the UR controller IP to 10.1.1.2

if ~exist('hwObj') && useHardware
    instruct = sprintf([...
        '\tPython module imported.\n',...
        '\tEnter server IP address: 10.1.1.5\n',...
        '\tEnter port: 30002\n',...
        '\tEnter number of connections to be made: 1\n',...
        '\tServer created.\n',...
        '\tBegin onboard controller, then press ENTER.\n',...
        '\tConnections established.\n',...
        '\tWould you like to create a URX connection as well? y\n',...
        '\tEnter URX address: 10.1.1.2\n',...
        '\tURX connection established.\n']);
    fprintf('PLEASE USE THE FOLLOWING RESPONSES:\n\n');
    fprintf(2,'%s\n\n',instruct)
    
    hwObj = UR;
end


%% Animate simulation and move the robot to test
% 홈포지션으로 가기 [0,-1.57,0,-1.57,0,0] 
% Home Position의 XYZ좌표는 (0,-191.45,1000)

% Home simulation
simObj.Home;

% simObj.FrameT(1,4)=[0];  % End-effector Frame부터의 Tool의 X위치 [mm]
% simObj.FrameT(2,4)=[0];  % End-effector Frame부터의 Tool의 Y위치 [mm]
% simObj.FrameT(3,4)=[0];  % End-effector Frame부터의 Tool의 Z위치 [mm]
grid()
drawnow
ToolPoseArray = [-1 0 0 0; 0 0 -1 -191.45; 0 -1 0 1000; 0 0 0 1];
pause(1)


%========================================================
% 제출시 반드시 아래 내용을 작성할 것
% 제출자 이름:             학번:               분반:
% 실험일자: 2021년   월   일 
% 본 코드를 자신의 노력으로 완료하였음을 확인함: 본인이름
%=========================================================


%% Linear path planning in Task Space

% Tool Position relative to the reference
% P1=[500, 400, 400];   % P = [x, y, z] for P1
% P2=[100, 200, 200];   % P = [x, y, z] for P2
% P3=[400  600  100];   % P = [x, y, z] for P3

% P1=[300, 400, 300];
% P2=[200, 300, 500];
% P3=[100  200  300];


P1=[400, 500, 400];
P2=[200, 300, 600];
P3=[100  200  400];


% Tool Orientation Angles used in Rotational Transformation Matrix relative to the reference
% th_P1 = [0, 0, 0]; % th_P = [thX, thY. thZ] for P1
% th_P3 = [pi/2, pi/2, 0]; % th_P = [thX, thY. thZ] for P3
% th_P2 = (th_P1+th_P3)/2; % th_P = [thX, thY. thZ] for P2

th_P1 = [pi/2, 0, 0]; % th_P = [thX, thY. thZ] for P1
th_P2 = [pi/2, pi/2, 0]; % th_P = [thX, thY. thZ] for P2
th_P3 = [0, pi/2, pi/2]; % th_P = [thX, thY. thZ] for P3



m=50;

% Path planning in Task Space
X=[linspace(P1(1),P2(1),m), linspace(P2(1),P3(1),m)];
Y=[linspace(P1(2),P2(2),m), linspace(P2(2),P3(2),m)];
Z=[linspace(P1(3),P2(3),m), linspace(P2(3),P3(3),m)];

% Rotational angles of x-, y-, and z-axes used in Transformation
thX=[linspace(th_P1(1),th_P2(1),m), linspace(th_P2(1),th_P3(1),m)];
thY=[linspace(th_P1(2),th_P2(2),m), linspace(th_P2(2),th_P3(2),m)];
thZ=[linspace(th_P1(3),th_P2(3),m), linspace(th_P2(3),th_P3(3),m)];

n=length(X);

% Animate the robot and Plot waypoints  

for i=1:n
    
    % Define the pose of the tool  
    
%     H_cur = Tx(X(i))*Ty(Y(i))*Tz(Z(i))*Ry(pi/2)*Rx(pi/2);
%     H_cur = Tx(X(i))*Ty(Y(i))*Tz(Z(i))*Ry(pi)*Rx(pi/2);
    H_cur = Tx(X(i))*Ty(Y(i))*Tz(Z(i))*Rx(thX(i))*Ry(thY(i))*Rz(thZ(i));
    simObj.ToolPose=H_cur;
%     q = simObj.Joints;    
    plt_Waypoints_TaskSpace = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.k');
    
    pause(0.5);

end


%% Path planning in Joint Space 


% Calculate Transformation Matrix and find joint angles
    H_p1 = Tx(P1(1))*Ty(P1(2))*Tz(P1(3))*Rx(th_P1(1))*Ry(th_P1(2))*Rz(th_P1(3));
    simObj.ToolPose=H_p1;
    q1=simObj.Joints;  % joint angles for P1 pose 
    

    H_p2 = Tx(P2(1))*Ty(P2(2))*Tz(P2(3))*Rx(th_P2(1))*Ry(th_P2(2))*Rz(th_P2(3));
    simObj.ToolPose=H_p2;
    q2=simObj.Joints  % joint angles for P2 pose 
    
    H_p3 = Tx(P3(1))*Ty(P3(2))*Tz(P3(3))*Rx(th_P3(1))*Ry(th_P3(2))*Rz(th_P3(3));
    simObj.ToolPose=H_p3;
    q3=simObj.Joints  % joint angles for P3 pose 

 
 q123=zeros(length(q1),n);   % Initilization of complate joint angles
 
    for i=1:length(q1)
                    
    q12(i,:)=linspace(q1(i),q2(i),m);
    q23(i,:)=linspace(q2(i),q3(i),m);
    
    end
    
    q123=[q12, q23];

% Animate the robot and Plot waypoints    
    for i=1:length(q123(1,:))
    
    simObj.Joints=q123(:,i);
    plt_Waypoints_JointSpace = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.g');
    pause(0.5);
    
    end

    
% The orgin of Tool relative to the reference
plot3(P1(1), P1(2), P1(3), 'kx');
plot3(P2(1), P2(2), P2(3), 'ko');
plot3(P3(1), P3(2), P3(3), 'k>');

Text_P1= "P1 (x) = "+ "[ " + int2str([P1(1), P1(2), P1(3)])+ " ] mm";
Text_P2= "P2 (o) = "+ "[ " + int2str([P2(1), P2(2), P2(3)])+ " ] mm";
Text_P3= "P3 (>) = "+ "[ " + int2str([P3(1), P3(2), P3(3)])+ " ] mm";
Text_z_axisTool= "Final rotation angle [\theta_X, \theta_Y, \theta_Z] = [" + string(rad2deg(th_P3(1))) +", " + string(rad2deg(th_P3(2))) + ", " + string(rad2deg(th_P3(3))) + " ] deg";  
% Text_z_axisTool= "Rotation angle is the same as the Home pose";  

text(P1(1), P1(2), P1(3)+50, Text_P1);
text(P2(1), P2(2), P2(3)+50, Text_P2);
text(P3(1), P3(2)+100, P3(3)-50, Text_P2);

text(P3(1), P3(2)+100, P3(3)-100, Text_z_axisTool);
    
%% 마무리

% simObj.Joints()
% simObj.ToolPose()
% simObj.ToolPose(1,4)
% simObj.ToolPose(2,4)
% simObj.ToolPose(3,4)
% plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.k');
% pause(0.5)
fprintf('Done')
