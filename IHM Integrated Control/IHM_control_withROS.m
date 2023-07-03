%% readme
% this one is the highest level control with every part integrated


%% task

clc;
clear all;

setenv('ROS_MASTER_URI','http://192.168.223.219:11311');
setenv('ROS_IP','192.168.223.81');
rosinit('192.168.223.219');

shapeSortingList = ["Square"]; % more shapes should be added to here
shapeSortingList_sz = size(shapeSortingList);

disp("Start Shape Sorting Task.")
disp("Below are the shapes we aim to sort");
disp(shapeSortingList)

%%

for sortingNum = 1:1:shapeSortingList_sz(1)
    
    
    fprintf("------Task start. Sorting shape: %s------\n", shapeSortingList(sortingNum))

    % disp("------Move to Picking Pose------")
    % arm_Move2PickPos()
    
    disp("------Picking------") % complete
    x = 3;
    y = 15;
    arm_Move2PickPos(x,y);
    pause(5)

    if gripper_pickup() == 1
        break;
    end

    start_y = arm_Move2ManipPos();
    disp(start_y)

    % disp("------Move to Manipulation Pose------")
    arm_Move2ManipPos()
    pause(5)
    
    %%
    disp("------Start Gripper Manipulation------") 
    start_coord = [0,7]; % 2x1

    goal_y = 10.0;


    end_pos = [0 goal_y 0]; % 3x1
    gripper_Move2GoalPos(start_coord,end_pos)
    % pause(2)


    
    % disp("------Adjusting Error------")
    % currentPos = vision_getPos();
    % error = goalPos - currentPos;
    % arm_adjustError(error)
    % pause(2)
    % 
    % 
    disp("------Move to Place Pose------")
    arm_Move2PlacePos(goal_y);
    pause(10)

    disp("------Releasing------") % complete
    gripper_release()
    % pause(2)
    
    % prompt = "wait keyboard press\n";
    % while input(prompt) == 1
    % end

    disp("------Task Completed------")

end

disp("------Complete Shape Sorting Task------")

rosshutdown;

% disp("------Back to Default Start Pose------")
% arm_Move2StartPose()
% pause(2)



