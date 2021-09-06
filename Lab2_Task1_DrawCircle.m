%% SCRIPT_DrawCircle
% This script demonstrates use of the URsim class capabilities with the UR
% class. The URsim object is used to visualize the robot and calculate
% inverse kinematics for the system. Waypoints are executed by getting
% joint angle from the simulation object and sending them to the
% URToolboxScript controller running on the UR control box.
%
%   M. Kutzer, 14Mar2018, USNA

clearvars -EXCEPT hwObj
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

%% Create path
% -> NOTE: This is provided for demonstration only. Perfect execution of
% this path will require infinite accelerations at the initial and final
% waypoints meaning the robot will not track perfectly.

% Define dependent variable
s = linspace(0,1,1000);
% Define circle radius (mm)
r = 100;
% Define position data
X = [];
X(1,:) = r*cos(2*pi*s); % x-position
X(2,:) = r*sin(2*pi*s); % y-position
X(3,:) = 0;             % z-position
X(4,:) = 1;             % Append 1 (homogeneous coordinate)



% Transform coordinates into the workspace of the robot
% X = Tz(500)*Rx(pi/2)*Tz(500)*X;    %  
% X = Tz(500)*Ty(-500)*Rx(pi/2)*X;     % same as above transformation
% X = Ty(-500)*Tz(500)*Rx(pi/2)*X;     % same as above transformation



%============================================================
% 제출시 반드시 아래 내용을 작성할 것
% 제출자 이름:             학번:               분반:
% 실험일자: 2021년   월   일 
% 본 코드를 자신의 노력으로 완료하였음을 확인함: 본인이름
%==============================================================

% determine the the transolation and orientation angles of the tool
% coordinate frame relative to the reference
Xc=0;
Yc=500;
Zc=500;

thX=pi/2;
thY=0;
thZ=0;

% Find Homogeneous Coordinate Transofrmation of the tool relatvie to the reference
T_trans=Ty(-500)*Tz(500);  % Transformation by sequential translations 
T_rot_xyz=Rx(thX)*Ry(thY)*Rz(thZ); % Transformation by sequential rotations
T_comb=T_trans*T_rot_xyz; % Combination of sequential translation and rotations as ordered here

% Find the path (in this case, the circle with the radius of 100 mm) of the tool relative to the reference (Homogeneous coordinates)
X = T_comb*X;   


% Plot waypoints
% plt_Waypoints = plot3(simObj.Axes,X(1,:),X(2,:),X(3,:),'.m');

%% Animate simulation and move the robot to test
% Home simulation
simObj.Home;
% Move robot to match simulation
if useHardware
    % Get joint position from the simulation
    q = simObj.Joints;
    % Send waypoint to the robot
    % -> Message syntax sends 6 joint positions and 6 joint velocities to
    % define a desired waypoint for the URToolboxScript controller. We are
    % assuming a desired velocity of zero for all joints at each waypoint.
    msg(hwObj,sprintf('(%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f)',q,zeros(6,1)));
    % Wait for the robot to finish executing the move
    UR_WaitForMove(hwObj);
end
% Allow plot to update
drawnow

% Move through waypoints
for i = 1:numel(s)
    % Define pose from waypoint
%     H_cur = Tx(X(1,i))*Ty(X(2,i))*Tz(X(3,i))*Rx(pi/2);
    H_cur = Tx(X(1,i))*Ty(X(2,i))*Tz(X(3,i))*Rx(thX)*Ry(thY)*Rz(thZ);
    % Set simulation toolpose to waypoint pose
    simObj.ToolPose = H_cur;
    
    % Move robot to match simulation
    q = simObj.Joints;
    if useHardware
        % Get joint position from the simulation
        q = simObj.Joints;
        % Send waypoint to the robot
        msg(hwObj,sprintf('(%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f)',q,zeros(6,1)));
        % Wait for move only on the first waypoint
        if i == 1
            % Wait for the robot to finish executing the move
            UR_WaitForMove(hwObj);
        end
    end
    
    % Allow plot to update
    drawnow;
    plt_Waypoints = plot3(simObj.Axes,X(1,i),X(2,i),X(3,i),'.m');
end
grid

%% Add some information in the plot

% The orgin of Tool relative to the reference
X_Tool = T_comb*[0; 0; 0; 1]; 
plot3(X_Tool(1), X_Tool(2), X_Tool(3), 'kx');

% Text_center_pos= append("The center of the circle : ", int2str([Xc Yc Zc]));
Text_center_pos= "Center of the circle (x) = "+ "[ " + int2str([Xc Yc Zc])+ " ] mm";
Text_z_axisTool= "Rotation angle [\theta_X, \theta_Y, \theta_Z] = [" + string(rad2deg(thX)) +", " + string(rad2deg(thY)) + ", " + string(rad2deg(thZ)) + " ] deg";  

% x_ann=[0.5 0.5];
% y_ann=[0.4 0.4];
% % z_ann=[Yc Yc-200*sign(Zc)];
% % 
% annotation('textarrow', x_ann, y_ann, 'String', center_pos)

text(Xc, Yc, Zc, Text_center_pos);
text(Xc, Yc, Zc+80, Text_z_axisTool);