clc
clear

% Load the robot
[robot, iRBT, tcpObj] = Load_Robot; % Add tcpObj for communication

% Points that the arm visits
wpts = [0.2 0.3 0.7;
        0.4 0.6 0.2;
        0.5 -0.3 0.3;
        0.2 0.3 0.7]';

% Calculate and plot the motion in 3D
Robot_motion(robot, iRBT, tcpObj, wpts); % Add tcpObj as input

% FUNCTIONS: %

% Load the Robot
function [robot, iRBT, tcpObj] = Load_Robot

    robot = loadrobot('universalUR5');
    robot.DataFormat = 'row';

    rotationX = @(t) [1 0 0; 0 cosd(t) -sind(t); 0 sind(t) cosd(t)];

    IHMgripperBody = rigidBody('gripper');
    addVisual(IHMgripperBody,"Mesh",'VFF_2022.STL')
    IHMgripperBodyJoint = rigidBodyJoint('IHMgripperBodyJoint','fixed');
    IHMgripperBody.Joint = IHMgripperBodyJoint;
    transfForIHMgripperBody = rotm2tform(rotationX(-90));
    setFixedTransform(IHMgripperBody.Joint, transfForIHMgripperBody)
    curEndEffectorBodyName = robot.BodyNames{10};
    addBody(robot,IHMgripperBody,curEndEffectorBodyName)

    transfForNewEndEffectorBody = rotm2tform(rotationX(180));
    transfForNewEndEffectorBody(:,4) = [0; -0.195; 0; 1];
    newEndEffectorBody = rigidBody('gripperEdge');
    setFixedTransform(newEndEffectorBody.Joint, transfForNewEndEffectorBody);
    IHMgripperBodyName = robot.BodyNames{11};
    addBody(robot,newEndEffectorBody,IHMgripperBodyName);

    


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Establish communication with the UR5e robot
    robotIP = '192.168.0.1'; % Replace with the actual IP address of the robot
    robotPort = 30002; % Replace with the actual port number of the robot
    tcpObj = tcpclient(robotIP, robotPort);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




    % Close the previous figure window before running the script again
    close(findobj('type','figure','name','Interactive Visualization'));

    iRBT = interactiveRigidBodyTree(robot);

end

% Perform Inverse Kinematics Calculations
function Inverse_Kin(desiredPosition, desiredRotation, robot, roboti, tcpObj)

    % home position
    q_home = [0 -90 0 -90 0 0]'*pi/180;

    % Pose Matrix 4x4
    desiredPose = trvec2tform(desiredPosition) * eul2tform(desiredRotation);

    % IK Calculation
    ikSol = inverseKinematics('RigidBodyTree', robot);
    ikSol.SolverParameters.AllowRandomRestart = false;
    ikWeights = [1 1 1 1 1 1];
    desiredJointAngles = ikSol('gripperEdge', double(desiredPose), ikWeights', q_home');

    % Send joint angles to the robot
    Send_Joint_Command(desiredJointAngles, tcpObj); % Send joint angles command

    Plot_Configuration(desiredJointAngles, roboti, robot);

end

% Send joint angles command to the robot
function Send_Joint_Command(jointAngles, tcpObj)
    command = ['movej(', num2str(jointAngles'), ', a=1.4, v=1.05)']; % URScript command
    fwrite(tcpObj, command); % Send the command to the robot
    pause(0.5); % Pause to allow the robot to process the command (adjust as needed)
end

% Plot the configuration for a given IK calculated pose
function Plot_Configuration(desiredJointAngles, roboti, robot)

    tform = getTransform(robot, desiredJointAngles, 'gripperEdge');
    position = tform(1:3, 4);

    q_home = [0 -90 0 -90 0 0]'*pi/180;

    rotate3d off;
    view(45,15)
    axis([-1 1 -1 1 -0.5 1])
    hold on
    zlim([-0.5 1.5])
    roboti.ShowMarker = false;
    roboti.Configuration = q_home; % joint angle space

    % Set robot configuration to the desired pose
    roboti.Configuration = desiredJointAngles;

    % Give endeffector Coordinates in title of plot
    title(['Endeffector Position: ', 'x = ', num2str(round(position(1, 1), 3)), ...
        ', y = ', num2str(round(position(2, 1), 3)), ', z = ', num2str(round(position(3, 1), 3))])

end

% Plot the motion of arm (many consecutive poses)
function Robot_motion(robot, iRBT, tcpObj, wpts)

    t_max = size(wpts,1);
    tpts = 0:t_max;
    tvec = 0:0.01:t_max;

    [q, ~, ~, ~] = cubicpolytraj(wpts, tpts, tvec);

    r = rateControl(200);
    iRBT.ShowMarker = false;  % Hide the marker

    showFigure(iRBT)

    for i = 1:size(q',1)
        Con = [q(:,i); zeros(3, size(q(:,i), 2))]';

        Inverse_Kin(Con(1,1:3), Con(1,4:6), robot, iRBT, tcpObj);
        if mod(i,4)==0
            scatter3(q(1,i), q(2,i), q(3,i));
            hold on
        end

        waitfor(r);
    end

    figure;
    plot(tvec, q)
    hold all
    plot(tpts, wpts, 'x')
    xlabel('t')
    ylabel('Positions')
    legend('X-positions','Y-positions', 'Z-Positions')
    hold off

end
