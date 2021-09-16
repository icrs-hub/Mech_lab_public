
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
        simObj.FrameT = Tz(160);
    
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

% simObj.FrameT(1,4)=[0];  % End-effector Frame부터의 Tool의 X위치 [mm]
% simObj.FrameT(2,4)=[0];  % End-effector Frame부터의 Tool의 Y위치 [mm]
% simObj.FrameT(3,4)=[0];  % End-effector Frame부터의 Tool의 Z위치 [mm]
grid()
drawnow
% T_home=simObj.Pose;
% ToolPoseArray = [-1  0   0      0; 
%                   0  0  -1 -191.45; 
%                   0 -1   0    1000; 
%                   0  0   0       1]; % Home pose

% Change orientation angle from the Home pose
ToolPoseArray = [ 0  1   0      0; 
                  0  0  -1 -191.45; 
                 -1  0   0    1000; 
                  0  0   0       1]; % Home pose

pause(1)

% Home Position의 XYZ좌표는 (0,-191.45,1000)

%학번 예시  끝번호 29이면
% 1번 포인트 x축에서 2번 좌표 (-500, -191.45, 675)
% 2번 포인트 z축에서 9번 좌표 (-550, -191.45, 475)
% 3번 포인트 xz평면에서 2,9번 좌표 (-500, -191.45, 475)

% 1번 포인트 ~ 2번 포인트 통과지점 9개 설정
% 2번 포인트 ~ 3번 포인트 통과지점 7개 설정

%============================================
% 제출시 반드시 아래 내용을 작성할 것
% 제출자 이름:             학번:               분반:
% 실험일자: 2021년   월   일 
% 본 코드를 자신의 노력으로 완료하였음을 확인함: 본인이름
%=============================================

% P1 좌표 (-500, -191.45, 675)
x = -500;
y = -191.45;
z = 675;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'k.');
pause(0.5)

% Add some information in the plot

% The orgin of Tool relative to the reference
plot3(x, y, z, 'kx');

% Text_center_pos= append("The center of the circle : ", int2str([Xc Yc Zc]));
Text_center_pos= "P1 (x) = "+ "[ " + int2str([x y z])+ " ] mm";
% Text_z_axisTool= "Rotation angle [\theta_X, \theta_Y, \theta_Z] = [" + string(thX) +", " + string(thY) + ", " + string(thZ) + " ] rad";  
Text_z_axisTool= "The tool frame is ratated from the Home pose about the fixed Y-axis by -pi/2 rad";  

text(x, y, z+50, Text_center_pos);
text(x, y, z+500, Text_z_axisTool);


% 1번 포인트 -> 1-1번 통과지점
x = -505;
y = -191.45;
z = 655;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'k.');
pause(0.5)

% 1-1번 통과지점 -> 1-2번 통과지점
x = -510;
y = -191.45;
z = 635;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'k.');
pause(0.5)

% 1-2번 통과지점 -> 1-3번 통과지점
x = -515;
y = -191.45;
z = 615;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'k.');
pause(0.5)

% 1-3번 통과지점 -> 1-4번 통과지점
x = -520;
y = -191.45;
z = 595;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'k.');
pause(0.5)

% 1-4번 통과지점 -> 1-5번 통과지점
x = -525;
y = -191.45;
z = 575;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'k.');
pause(0.5)

% 1-5번 통과지점 -> 1-6번 통과지점
x = -530;
y = -191.45;
z = 555;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'k.');
pause(0.5)

% 1-6번 통과지점 -> 1-7번 통과지점
x = -535;
y = -191.45;
z = 535;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'k.');
pause(0.5)

% 1-7번 통과지점 -> 1-8번 통과지점
x = -540;
y = -191.45;
z = 515;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'k.');
pause(0.5)

% 1-8번 통과지점 -> 1-9번 통과지점
x = -545;
y = -191.45;
z = 495;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'k.');
pause(0.5)

% 1-9번 통과지점 -> 2번 포인트
x = -550;
y = -191.45;
z = 475;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'k.');
pause(0.5)

% The orgin of Tool relative to the reference
plot3(x, y, z, 'ko');

% Text_center_pos= append("The center of the circle : ", int2str([Xc Yc Zc]));
Text_center_pos= "P2 (O) = "+ "[ " + int2str([x y z])+ " ] mm";
% Text_z_axisTool= "Rotation angle [\theta_X, \theta_Y, \theta_Z] = [" + string(thX) +", " + string(thY) + ", " + string(thZ) + " ] rad";  
% Text_z_axisTool= "Rotation angle is the same as the Home pose";  

text(x+100, y+100, z+300, Text_center_pos);
% text(x, y, z+80, Text_z_axisTool);

% 2번 포인트 -> 2-1번 통과지점
x = -543.75;
y = -191.45;
z = 475;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'k.');
pause(0.5)

% 2-1번 통과지점 -> 2-2번 통과지점
x = -537.5;
y = -191.45;
z = 475;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'k.');
pause(0.5)

% 2-2번 통과지점 -> 2-3번 통과지점
x = -531.25;
y = -191.45;
z = 475;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'k.');
pause(0.5)

% 2-3번 통과지점 -> 2-4번 통과지점
x = -525;
y = -191.45;
z = 475;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'k.');
pause(0.5)

% 2-4번 통과지점 -> 2-5번 통과지점
x = -518.75;
y = -191.45;
z = 475;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'k.');
pause(0.5)

% 2-5번 통과지점 -> 2-6번 통과지점
x = -512.5;
y = -191.45;
z = 475;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'k.');
pause(0.5)

% 2-6번 통과지점 -> 2-7번 통과지점
x = -506.25;
y = -191.45;
z = 475;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'k.');
pause(0.5)

% 2-7번 통과지점 -> 3번 포인트
x = -500;
y = -191.45;
z = 475;
ToolPoseArray(1,4) = x;
ToolPoseArray(2,4) = y;
ToolPoseArray(3,4) = z;
simObj.ToolPose = ToolPoseArray;
plt_Waypoints = plot3(simObj.Axes,simObj.ToolPose(1,4),simObj.ToolPose(2,4),simObj.ToolPose(3,4),'k.');
pause(0.5)

% The orgin of Tool relative to the reference
plot3(x, y, z, 'k>');

% Text_center_pos= append("The center of the circle : ", int2str([Xc Yc Zc]));
Text_center_pos= "P3 (>) = "+ "[ " + int2str([x y z])+ " ] mm";
% Text_z_axisTool= "Rotation angle [\theta_X, \theta_Y, \theta_Z] = [" + string(thX) +", " + string(thY) + ", " + string(thZ) + " ] rad";  
% Text_z_axisTool= "Rotation angle is the same as the Home pose"; 

text(x+150, y+150, z+400, Text_center_pos);
% text(x, y, z+80, Text_z_axisTool);

simObj.Joints()
simObj.ToolPose()
simObj.ToolPose(1,4)
simObj.ToolPose(2,4)
simObj.ToolPose(3,4)
fprintf('Done')
