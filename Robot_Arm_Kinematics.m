clc
clear

% Load the Robot
robot = loadrobot('universalUR5e', 'DataFormat', 'column');

%% Forward Kinematics for Desired Angles

desiredJointAngles = [0, 0, 0, 0, 0, 0];
Forward_Kinematics(desiredJointAngles, robot);

%% Inverse Kinematics for Desired Endeffector Coordinates

desiredPosition = [0.6, 0.6, 0.2];
desiredRotation = eye(3);
desiredPose = [desiredRotation, desiredPosition'; 0, 0, 0, 1];
Inverse_Kinematics(desiredPose, robot);


%% Functions
function Forward_Kinematics(desiredJointAngles, robot)
    tform = getTransform(robot, desiredJointAngles(:), 'tool0');
    Plot_Configuration(desiredJointAngles, tform, robot)
end


function Inverse_Kinematics(desiredPose, robot)

    ikSol = inverseKinematics('RigidBodyTree', robot);
    initialGuess = robot.homeConfiguration;
    desiredJointAngles = ikSol('tool0', desiredPose, [1, 1, 1, 1, 1, 1], initialGuess);

    % Apply joint constraint for Joint 2
    %desiredJointAngles(2) = max(-pi, min(desiredJointAngles(2), 0));

    tform = getTransform(robot, desiredJointAngles(:), 'tool0');
    Plot_Configuration(desiredJointAngles, tform, robot)
end


function Plot_Configuration(desiredJointAngles, tform, robot)
    position = tform(1:3, 4);
    figure;
    hold on;
    show(robot, desiredJointAngles(:), 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on');
    lighting gouraud;
    light('Position', [0, 0, 10], 'Style', 'infinite');
    material('dull');
    view(3);
    axis equal;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    grid on;
    title(['Endeffector Position: ', 'x = ', num2str(round(position(1, 1), 3)), ...
        ', y = ', num2str(round(position(2, 1), 3)), ', z = ', num2str(round(position(3, 1), 3))])
end
