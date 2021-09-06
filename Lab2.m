
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
% Home simulation
simObj.Home;

simObj.FrameT(1,4)=[0];  % End-effector Frame부터의 Tool의 X위치 [mm]
simObj.FrameT(2,4)=[0];  % End-effector Frame부터의 Tool의 Y위치 [mm]
simObj.FrameT(3,4)=[0];  % End-effector Frame부터의 Tool의 Z위치 [mm]
grid()
drawnow
ToolPoseArray = [-1 0 0 0; 0 0 -1 -191.45; 0 -1 0 1000; 0 0 0 1];
pause(1)
t = linspace(1,60,60);
w = 2*pi/numel(t);

%============================================
% 제출시 반드시 아래 내용을 작성할 것
% 제출자 이름:             학번:               분반:
% 실험일자: 2021년   월   일 
% 본 코드를 자신의 노력으로 완료하였음을 확인함: 본인이름
%=============================================



% Home Position의 XYZ좌표는 (0,-191.45,1000)
%% (예제 내용) Tool 좌표계 +X, +Y, +Z축을 Base의 -X, -Z, -Y축에 정렬,
%	Vx=50*sin(w*t), Vy=50*sin(w*t)에서 적분

for count = t
   x = 200 + 50*(-cos(w*count));
   y = -350 + 50*sin(w*count);
   z = 300;
ToolPoseArray = [ -1 0 0 x
                  0 0 -1 y
                  0 -1 0 z
                  0 0 0 1];
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.k');
pause(0.5)
end

%% 마무리

simObj.Joints()
simObj.ToolPose()
simObj.ToolPose(1,4)
simObj.ToolPose(2,4)
simObj.ToolPose(3,4)
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.k');
pause(0.5)
fprintf('Done')