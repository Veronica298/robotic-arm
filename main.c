clc;            % Clear the command window
clear;          % Delete all variables from workspace
close all;      % Close all open figure windows

%% ========== ARDUINO SETUP ==========
a = arduino('COM5','Uno','Libraries','Servo'); 
% Connect MATLAB to Arduino on COM5 and enable Servo library

s1 = servo(a,'D3');   % Attach servo 1 to digital pin D3
s2 = servo(a,'D4');   % Attach servo 2 to digital pin D4
s3 = servo(a,'D5');   % Attach servo 3 to digital pin D5

%% ========== ROBOT MODEL ==========
L0 = 0.07;           
% Height from base of the robot to first joint

La1 = 0.1;            
% Length of first arm

Lb1 = 0.01;           
% Offset of second joint

L2 = 0.12;            
% Length of second arm

L3 = 0.21;            
% Length of third arm

% DH Parameters format:
% [r     alpha     d     theta]
dhparams = [0    0      L0   0;
            Lb1  pi/2   La1  0;
            L2  -pi     0    0;
            L3   0      0    0];
% Define all DH parameters in a table

robot = rigidBodyTree; 
% Create empty robot model structure

% -------- FIRST BODY (FIXED BASE) --------
body1 = rigidBody('body1');                     
jnt1 = rigidBodyJoint('jnt1','fixed');        
% First joint does not move (base is fixed)

setFixedTransform(jnt1,dhparams(1,:),'dh');   
% Assign first row of DH table to joint1

body1.Joint = jnt1;                           
addBody(robot,body1,'base');                  
% Attach body1 to the robot base

% -------- SECOND BODY --------
body2 = rigidBody('body2');                  
jnt2 = rigidBodyJoint('jnt2','revolute');   
% Second joint is rotating

setFixedTransform(jnt2,dhparams(2,:),'dh');  
body2.Joint = jnt2;
addBody(robot,body2,'body1');                
% Attach body2 to body1

% -------- THIRD BODY --------
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');    
% Third rotating joint

setFixedTransform(jnt3,dhparams(3,:),'dh');  
body3.Joint = jnt3;
addBody(robot,body3,'body2');

% -------- END EFFECTOR --------
endeff = rigidBody('endeffector');            
jnt4 = rigidBodyJoint('jnt4','revolute');    
% Final rotating joint

setFixedTransform(jnt4,dhparams(4,:),'dh');
endeff.Joint = jnt4;
addBody(robot,endeff,'body3');               
% Attach end effector

%% ========== INITIAL POSE ==========
config = homeConfiguration(robot);           
% Get default joint configuration

config(1).JointPosition = 0;                
config(2).JointPosition = 0;
config(3).JointPosition = 0;
% Set all joints to 0 rad initial position

showdetails(robot);                         
% Display structure of robot in command window

hFigure = show(robot,config);               
% Display robot graphically

view(3); grid on;                           
% 3D view + grid
hold on                                    

%% ========== SQUARE TRAJECTORY ==========
x = 0; y = 0.6; z = 0.4;                   
% Starting point of square (end effector position)

side = 0.04;                                
% Length of one side of the square

t0 = trvec2tform([x y z]);                  
% First corner

t1 = trvec2tform([x+side y z]);             
% Second corner

t2 = trvec2tform([x+side y z-side]);        
% Third corner

t3 = trvec2tform([x y z-side]);             
% Fourth corner

tInterval = [0 1];                          
% Motion time from 0 to 1 second

tvec = 0:0.01:1;                            
% Time divided into 100 steps

% Generate smooth motion between points
[tf1,~,~] = transformtraj(t0,t1,tInterval,tvec);
[tf2,~,~] = transformtraj(t1,t2,tInterval,tvec);
[tf3,~,~] = transformtraj(t2,t3,tInterval,tvec);
[tf4,~,~] = transformtraj(t3,t0,tInterval,tvec);

Traj = {tf1, tf2, tf3, tf4};                
% Store all trajectory edges

%% ===== REACHABILITY CHECK =====
ik = inverseKinematics('RigidBodyTree',robot);
% Create inverse kinematics solver

weights = [0 0 0 1 1 1];                     
% Only care about position accuracy, ignore orientation

initialguess = homeConfiguration(robot);    

points = {t0, t1, t2, t3};                  
pointNames = ["P1", "P2", "P3", "P4"];

reachable = true;

maxReach = La1 + L2 + L3;                    
% Max distance robot arm can reach theoretically

disp("===== REACHABILITY CHECK RESULTS =====");

for i = 1:4
    tcp = tform2trvec(points{i});          
    % Extract XYZ coordinates

    base = [0 0 L0];                        
    % Base position

    distance = norm(tcp - base);           
    % Euclidean distance from base

    [configSol, solInfo] = ik('endeffector', points{i}, weights, initialguess);

    if solInfo.ExitFlag <= 0 || distance > maxReach
        fprintf("❌ %s is NOT reachable (Distance = %.3f m)\n", pointNames(i), distance);
        reachable = false;
    else
        fprintf("✅ %s is reachable (Distance = %.3f m)\n", pointNames(i), distance);
    end
end

if reachable == false
    error("❗ Square trajectory canceled: One or more points are unreachable.");
else
    disp("✅ All points are reachable. Square trajectory will start.");
end

%% ========== DRAW THE SQUARE ==========
initialguess = config;

% Create plotting line for drawing square path
hPlot = plot3(NaN, NaN, NaN, 'r-', 'LineWidth', 2);

squarePathX = [];
squarePathY = [];
squarePathZ = [];

for edge = 1:4                            
    T = Traj{edge};                       

    for i = 1:101
        tform = T(:,:,i);                

        [configSoln, ~] = ik('endeffector', tform, weights, initialguess);

        % Convert from radians to degrees
        t1 = rad2deg(configSoln(1).JointPosition);
        t2 = rad2deg(configSoln(2).JointPosition);
        t3 = rad2deg(configSoln(3).JointPosition);

        % Limit angles to servo safe range
        t1 = max(0, min(180, t1));
        t2 = max(0, min(180, t2));
        t3 = max(-90, min(90, t3));

        % Normalize angles from (0–180) for Arduino servo input
        a1 = t1 / 180;
        a2 = t2 / 180;
        a3 = (t3 + 90) / 180;

        % Send angles to real Arduino servos
        writePosition(s1, a1);
        writePosition(s2, a2);
        writePosition(s3, a3);

        % Update robot visual
        show(robot, configSoln, 'PreservePlot', false);

        % Draw square path
        tcp = tform2trvec(tform);

        squarePathX(end+1) = tcp(1);
        squarePathY(end+1) = tcp(2);
        squarePathZ(end+1) = tcp(3);

        set(hPlot, 'XData', squarePathX,...
                   'YData', squarePathY,...
                   'ZData', squarePathZ);

        drawnow;                          
        % Force screen update

        initialguess = configSoln;        
        % Improve IK convergence

        pause(0.03);                     
        % Slow for visibility + servo response
    end
end
