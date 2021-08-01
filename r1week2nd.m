
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

% Home Position의 XYZ좌표는 (0,-191.45,1000)
%% ㅎ 그리기 ( x = 191.45mm 일정 )
slice = linspace(1,10,10);
x_1 = 191.45; % 시작지점값
x_2 = 191.45; % 도착지점값
y_1 = -350;   % 시작지점값
y_2 = -350;   % 도착지점값
z_1 = 675;    % 시작지점값
z_2 = 650;    % 도착지점값
for count = slice
   x_d = (x_2 - x_1)/numel(slice);
   y_d = (y_2 - y_1)/numel(slice);
   z_d = (z_2 - z_1)/numel(slice);
   x = x_1 + x_d * count;
   y = y_1 + y_d * count;
   z = z_1 + z_d * count;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.k');
pause(0.5)
end

slice = linspace(1,10,10);
x_1 = 191.45; % 시작지점값
x_2 = 191.45; % 도착지점값
y_1 = -400;   % 시작지점값
y_2 = -300;   % 도착지점값
z_1 = 650;    % 시작지점값
z_2 = 650;    % 도착지점값
for count = slice
   x_d = (x_2 - x_1)/numel(slice);
   y_d = (y_2 - y_1)/numel(slice);
   z_d = (z_2 - z_1)/numel(slice);
   x = x_1 + x_d * count;
   y = y_1 + y_d * count;
   z = z_1 + z_d * count;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.k');
pause(0.5)
end

slice = linspace(1,10,10);
x_1 = 191.45; % 시작지점값
x_2 = 191.45; % 도착지점값
y_1 = -350;   % 시작지점값
y_2 = -400;   % 도착지점값
z_1 = 625;    % 시작지점값
z_2 = 600;    % 도착지점값
for count = slice
   x_d = (x_2 - x_1)/numel(slice);
   y_d = (y_2 - y_1)/numel(slice);
   z_d = (z_2 - z_1)/numel(slice);
   x = x_1 + x_d * count;
   y = y_1 + y_d * count;
   z = z_1 + z_d * count;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.k');
pause(0.5)
end

slice = linspace(1,10,10);
x_1 = 191.45; % 시작지점값
x_2 = 191.45; % 도착지점값
y_1 = -400;   % 시작지점값
y_2 = -350;   % 도착지점값
z_1 = 600;    % 시작지점값
z_2 = 575;    % 도착지점값
for count = slice
   x_d = (x_2 - x_1)/numel(slice);
   y_d = (y_2 - y_1)/numel(slice);
   z_d = (z_2 - z_1)/numel(slice);
   x = x_1 + x_d * count;
   y = y_1 + y_d * count;
   z = z_1 + z_d * count;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.k');
pause(0.5)
end

slice = linspace(1,10,10);
x_1 = 191.45; % 시작지점값
x_2 = 191.45; % 도착지점값
y_1 = -350;   % 시작지점값
y_2 = -300;   % 도착지점값
z_1 = 575;    % 시작지점값
z_2 = 600;    % 도착지점값
for count = slice
   x_d = (x_2 - x_1)/numel(slice);
   y_d = (y_2 - y_1)/numel(slice);
   z_d = (z_2 - z_1)/numel(slice);
   x = x_1 + x_d * count;
   y = y_1 + y_d * count;
   z = z_1 + z_d * count;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.k');
pause(0.5)
end

slice = linspace(1,10,10);
x_1 = 191.45; % 시작지점값
x_2 = 191.45; % 도착지점값
y_1 = -300;   % 시작지점값
y_2 = -350;   % 도착지점값
z_1 = 600;    % 시작지점값
z_2 = 625;    % 도착지점값
for count = slice
   x_d = (x_2 - x_1)/numel(slice);
   y_d = (y_2 - y_1)/numel(slice);
   z_d = (z_2 - z_1)/numel(slice);
   x = x_1 + x_d * count;
   y = y_1 + y_d * count;
   z = z_1 + z_d * count;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.k');
pause(0.5)
end

%% ㅏ 그리기 ( x = 191.45mm 일정 )

slice = linspace(1,10,10);
x_1 = 191.45; % 시작지점값
x_2 = 191.45; % 도착지점값
y_1 = -250;   % 시작지점값
y_2 = -250;   % 도착지점값
z_1 = 650;    % 시작지점값
z_2 = 575;    % 도착지점값
for count = slice
   x_d = (x_2 - x_1)/numel(slice);
   y_d = (y_2 - y_1)/numel(slice);
   z_d = (z_2 - z_1)/numel(slice);
   x = x_1 + x_d * count;
   y = y_1 + y_d * count;
   z = z_1 + z_d * count;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.k');
pause(0.5)
end

slice = linspace(1,10,10);
x_1 = 191.45; % 시작지점값
x_2 = 191.45; % 도착지점값
y_1 = -250;   % 시작지점값
y_2 = -200;   % 도착지점값
z_1 = 600;    % 시작지점값
z_2 = 600;    % 도착지점값
for count = slice
   x_d = (x_2 - x_1)/numel(slice);
   y_d = (y_2 - y_1)/numel(slice);
   z_d = (z_2 - z_1)/numel(slice);
   x = x_1 + x_d * count;
   y = y_1 + y_d * count;
   z = z_1 + z_d * count;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.k');
pause(0.5)
end

%% ㄴ 그리기 ( x = 191.45mm 일정 )

slice = linspace(1,10,10);
x_1 = 191.45; % 시작지점값
x_2 = 191.45; % 도착지점값
y_1 = -400;   % 시작지점값
y_2 = -400;   % 도착지점값
z_1 = 550;    % 시작지점값
z_2 = 500;    % 도착지점값
for count = slice
   x_d = (x_2 - x_1)/numel(slice);
   y_d = (y_2 - y_1)/numel(slice);
   z_d = (z_2 - z_1)/numel(slice);
   x = x_1 + x_d * count;
   y = y_1 + y_d * count;
   z = z_1 + z_d * count;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.k');
pause(0.5)
end

slice = linspace(1,10,10);
x_1 = 191.45; % 시작지점값
x_2 = 191.45; % 도착지점값
y_1 = -400;   % 시작지점값
y_2 = -250;   % 도착지점값
z_1 = 500;    % 시작지점값
z_2 = 500;    % 도착지점값
for count = slice
   x_d = (x_2 - x_1)/numel(slice);
   y_d = (y_2 - y_1)/numel(slice);
   z_d = (z_2 - z_1)/numel(slice);
   x = x_1 + x_d * count;
   y = y_1 + y_d * count;
   z = z_1 + z_d * count;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.k');
pause(0.5)
end

%% ㅂ 그리기  ( y = -191.45mm 일정 )

slice = linspace(1,10,10);
x_1 = -400; % 시작지점값
x_2 = -400; % 도착지점값
y_1 = -191.45;   % 시작지점값
y_2 = -191.45;   % 도착지점값
z_1 = 675;    % 시작지점값
z_2 = 600;    % 도착지점값
for count = slice
   x_d = (x_2 - x_1)/numel(slice);
   y_d = (y_2 - y_1)/numel(slice);
   z_d = (z_2 - z_1)/numel(slice);
   x = x_1 + x_d * count;
   y = y_1 + y_d * count;
   z = z_1 + z_d * count;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.k');
pause(0.5)
end

slice = linspace(1,10,10);
x_1 = -300; % 시작지점값
x_2 = -300; % 도착지점값
y_1 = -191.45;   % 시작지점값
y_2 = -191.45;   % 도착지점값
z_1 = 675;    % 시작지점값
z_2 = 600;    % 도착지점값
for count = slice
   x_d = (x_2 - x_1)/numel(slice);
   y_d = (y_2 - y_1)/numel(slice);
   z_d = (z_2 - z_1)/numel(slice);
   x = x_1 + x_d * count;
   y = y_1 + y_d * count;
   z = z_1 + z_d * count;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.k');
pause(0.5)
end

slice = linspace(1,10,10);
x_1 = -400; % 시작지점값
x_2 = -300; % 도착지점값
y_1 = -191.45;   % 시작지점값
y_2 = -191.45;   % 도착지점값
z_1 = 625;    % 시작지점값
z_2 = 625;    % 도착지점값
for count = slice
   x_d = (x_2 - x_1)/numel(slice);
   y_d = (y_2 - y_1)/numel(slice);
   z_d = (z_2 - z_1)/numel(slice);
   x = x_1 + x_d * count;
   y = y_1 + y_d * count;
   z = z_1 + z_d * count;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.k');
pause(0.5)
end

slice = linspace(1,10,10);
x_1 = -400; % 시작지점값
x_2 = -300; % 도착지점값
y_1 = -191.45;   % 시작지점값
y_2 = -191.45;   % 도착지점값
z_1 = 600;    % 시작지점값
z_2 = 600;    % 도착지점값
for count = slice
   x_d = (x_2 - x_1)/numel(slice);
   y_d = (y_2 - y_1)/numel(slice);
   z_d = (z_2 - z_1)/numel(slice);
   x = x_1 + x_d * count;
   y = y_1 + y_d * count;
   z = z_1 + z_d * count;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.k');
pause(0.5)
end

%% ㅏ 그리기  ( y = -191.45mm 일정 )

slice = linspace(1,10,10);
x_1 = -250; % 시작지점값
x_2 = -250; % 도착지점값
y_1 = -191.45;   % 시작지점값
y_2 = -191.45;   % 도착지점값
z_1 = 675;    % 시작지점값
z_2 = 575;    % 도착지점값
for count = slice
   x_d = (x_2 - x_1)/numel(slice);
   y_d = (y_2 - y_1)/numel(slice);
   z_d = (z_2 - z_1)/numel(slice);
   x = x_1 + x_d * count;
   y = y_1 + y_d * count;
   z = z_1 + z_d * count;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.k');
pause(0.5)
end


slice = linspace(1,10,10);
x_1 = -250; % 시작지점값
x_2 = -200; % 도착지점값
y_1 = -191.45;   % 시작지점값
y_2 = -191.45;   % 도착지점값
z_1 = 625;    % 시작지점값
z_2 = 625;    % 도착지점값
for count = slice
   x_d = (x_2 - x_1)/numel(slice);
   y_d = (y_2 - y_1)/numel(slice);
   z_d = (z_2 - z_1)/numel(slice);
   x = x_1 + x_d * count;
   y = y_1 + y_d * count;
   z = z_1 + z_d * count;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.k');
pause(0.5)
end

%% ㅌ 그리기  ( y = -191.45mm 일정 )

slice = linspace(1,10,10);
x_1 = -400; % 시작지점값
x_2 = -250; % 도착지점값
y_1 = -191.45;   % 시작지점값
y_2 = -191.45;   % 도착지점값
z_1 = 550;    % 시작지점값
z_2 = 550;    % 도착지점값
for count = slice
   x_d = (x_2 - x_1)/numel(slice);
   y_d = (y_2 - y_1)/numel(slice);
   z_d = (z_2 - z_1)/numel(slice);
   x = x_1 + x_d * count;
   y = y_1 + y_d * count;
   z = z_1 + z_d * count;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.k');
pause(0.5)
end

slice = linspace(1,10,10);
x_1 = -400; % 시작지점값
x_2 = -250; % 도착지점값
y_1 = -191.45;   % 시작지점값
y_2 = -191.45;   % 도착지점값
z_1 = 525;    % 시작지점값
z_2 = 525;    % 도착지점값
for count = slice
   x_d = (x_2 - x_1)/numel(slice);
   y_d = (y_2 - y_1)/numel(slice);
   z_d = (z_2 - z_1)/numel(slice);
   x = x_1 + x_d * count;
   y = y_1 + y_d * count;
   z = z_1 + z_d * count;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.k');
pause(0.5)
end

slice = linspace(1,10,10);
x_1 = -400; % 시작지점값
x_2 = -250; % 도착지점값
y_1 = -191.45;   % 시작지점값
y_2 = -191.45;   % 도착지점값
z_1 = 500;    % 시작지점값
z_2 = 500;    % 도착지점값
for count = slice
   x_d = (x_2 - x_1)/numel(slice);
   y_d = (y_2 - y_1)/numel(slice);
   z_d = (z_2 - z_1)/numel(slice);
   x = x_1 + x_d * count;
   y = y_1 + y_d * count;
   z = z_1 + z_d * count;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.k');
pause(0.5)
end

slice = linspace(1,10,10);
x_1 = -400; % 시작지점값
x_2 = -400; % 도착지점값
y_1 = -191.45;   % 시작지점값
y_2 = -191.45;   % 도착지점값
z_1 = 550;    % 시작지점값
z_2 = 500;    % 도착지점값
for count = slice
   x_d = (x_2 - x_1)/numel(slice);
   y_d = (y_2 - y_1)/numel(slice);
   z_d = (z_2 - z_1)/numel(slice);
   x = x_1 + x_d * count;
   y = y_1 + y_d * count;
   z = z_1 + z_d * count;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.k');
pause(0.5)
end

%% ㄷ 그리기 ( x = -191.45mm 일정 )

slice = linspace(1,10,10);
x_1 = -191.45; % 시작지점값
x_2 = -191.45; % 도착지점값
y_1 = 400;   % 시작지점값
y_2 = 300;   % 도착지점값
z_1 = 650;    % 시작지점값
z_2 = 650;    % 도착지점값
for count = slice
   x_d = (x_2 - x_1)/numel(slice);
   y_d = (y_2 - y_1)/numel(slice);
   z_d = (z_2 - z_1)/numel(slice);
   x = x_1 + x_d * count;
   y = y_1 + y_d * count;
   z = z_1 + z_d * count;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.k');
pause(0.5)
end

slice = linspace(1,10,10);
x_1 = -191.45; % 시작지점값
x_2 = -191.45; % 도착지점값
y_1 = 400;   % 시작지점값
y_2 = 300;   % 도착지점값
z_1 = 600;    % 시작지점값
z_2 = 600;    % 도착지점값
for count = slice
   x_d = (x_2 - x_1)/numel(slice);
   y_d = (y_2 - y_1)/numel(slice);
   z_d = (z_2 - z_1)/numel(slice);
   x = x_1 + x_d * count;
   y = y_1 + y_d * count;
   z = z_1 + z_d * count;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.k');
pause(0.5)
end


slice = linspace(1,10,10);
x_1 = -191.45; % 시작지점값
x_2 = -191.45; % 도착지점값
y_1 = 400;   % 시작지점값
y_2 = 400;   % 도착지점값
z_1 = 650;    % 시작지점값
z_2 = 600;    % 도착지점값
for count = slice
   x_d = (x_2 - x_1)/numel(slice);
   y_d = (y_2 - y_1)/numel(slice);
   z_d = (z_2 - z_1)/numel(slice);
   x = x_1 + x_d * count;
   y = y_1 + y_d * count;
   z = z_1 + z_d * count;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.k');
pause(0.5)
end

%% ㅐ 그리기 ( x = -191.45mm 일정 )


slice = linspace(1,10,10);
x_1 = -191.45; % 시작지점값
x_2 = -191.45; % 도착지점값
y_1 = 250;   % 시작지점값
y_2 = 250;   % 도착지점값
z_1 = 650;    % 시작지점값
z_2 = 550;    % 도착지점값
for count = slice
   x_d = (x_2 - x_1)/numel(slice);
   y_d = (y_2 - y_1)/numel(slice);
   z_d = (z_2 - z_1)/numel(slice);
   x = x_1 + x_d * count;
   y = y_1 + y_d * count;
   z = z_1 + z_d * count;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.k');
pause(0.5)
end

slice = linspace(1,10,10);
x_1 = -191.45; % 시작지점값
x_2 = -191.45; % 도착지점값
y_1 = 200;   % 시작지점값
y_2 = 200;   % 도착지점값
z_1 = 650;    % 시작지점값
z_2 = 550;    % 도착지점값
for count = slice
   x_d = (x_2 - x_1)/numel(slice);
   y_d = (y_2 - y_1)/numel(slice);
   z_d = (z_2 - z_1)/numel(slice);
   x = x_1 + x_d * count;
   y = y_1 + y_d * count;
   z = z_1 + z_d * count;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.k');
pause(0.5)
end

slice = linspace(1,10,10);
x_1 = -191.45; % 시작지점값
x_2 = -191.45; % 도착지점값
y_1 = 250;   % 시작지점값
y_2 = 200;   % 도착지점값
z_1 = 600;    % 시작지점값
z_2 = 600;    % 도착지점값
for count = slice
   x_d = (x_2 - x_1)/numel(slice);
   y_d = (y_2 - y_1)/numel(slice);
   z_d = (z_2 - z_1)/numel(slice);
   x = x_1 + x_d * count;
   y = y_1 + y_d * count;
   z = z_1 + z_d * count;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.k');
pause(0.5)
end

%% 마무리

simObj.Home;

simObj.Joints()
simObj.ToolPose()
simObj.ToolPose(1,4)
simObj.ToolPose(2,4)
simObj.ToolPose(3,4)
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'.k');
pause(0.5)
fprintf('Done')