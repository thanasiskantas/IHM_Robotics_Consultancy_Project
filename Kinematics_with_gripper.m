clc
clear

% Load the Robot
robot = loadrobot('universalUR5e');
robot.DataFormat = 'row';

% Define the gripper link
gripperLink = robotics.RigidBody('gripper_link');
row_gripperLink = collisionCylinder(0.1,0.2); % cylinder: radius,length
row_gripperLink.Pose = trvec2tform([0 0 0.2/2]);
addCollision(gripperLink,row_gripperLink);

% Define the gripper Joint
gripperJoint = robotics.Joint('gripper_joint', 'fixed');
gripperJoint.setFixedTransform(eye(4));
gripperLink.Joint = gripperJoint;

% Add the gripper link to the robot
robot.addBody(gripperLink, 'tool0');

% Create an interactive robot representation
roboti = interactiveRigidBodyTree(robot);
ax = gca;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
desiredPosition = [0.1 0.4 0.1];
desiredRotation = [0 0 0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


Inverse_Kin(desiredPosition, desiredRotation, robot, roboti)


function Inverse_Kin(desiredPosition, desiredRotation, robot, roboti)

    % home position
    q_home = [0 -90 0 -90 0 0]'*pi/180;

    % Pose Matrix 4x4
    desiredPose = trvec2tform(desiredPosition) * eul2tform(desiredRotation);

    % IK Calculation
    ikSol = inverseKinematics('RigidBodyTree', robot);
    ikSol.SolverParameters.AllowRandomRestart = false;
    ikWeights = [1 1 1 1 1 1];
    desiredJointAngles = ikSol('gripper_link', desiredPose, ikWeights', q_home');

    Plot_Configuration(desiredJointAngles, roboti, robot)
end


function Plot_Configuration(desiredJointAngles, roboti, robot)

    tform = getTransform(robot, desiredJointAngles, 'gripper_link');
    position = tform(1:3, 4);

    q_home = [0 -90 0 -90 0 0]'*pi/180;

    rotate3d off;
    view(145,25)
    lightangle(20,-160)
    axis([-1 1 -1 1 -0.5 1])
    hold on
    zlim([-0.5 1.5])
    roboti.ShowMarker = false;
    roboti.Configuration = q_home; % joint angle space

    % Set robot configuration to the desired pose
    roboti.Configuration = desiredJointAngles;
    roboti.ShowMarker = true;

    % Give end effector coordinates in the title of the plot
    title(['End Effector Position: ', 'x = ', num2str(round(position(1, 1), 3)), ...
        ', y = ', num2str(round(position(2, 1), 3)), ', z = ', num2str(round(position(3, 1), 3))])
end

