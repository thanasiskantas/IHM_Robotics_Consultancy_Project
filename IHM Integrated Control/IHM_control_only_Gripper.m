 %% readme
% this one is the highest level control with every part integrated


%% task

clc;
clear all;

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
    if gripper_pickup() == 1
        break;
    end
    
    prompt = "wait keyboard press";
    while input(prompt) == 1
    end

    % disp("------Move to Manipulation Pose------")
    % arm_Move2ManipPos()
    % pause(2)
    
    %%
    disp("------Start Gripper Manipulation------") 
    start_coord = [0 8]; % 2x1
    end_pos = [0 10 45]; % 3x1
    gripper_Move2GoalPos_shape2(start_coord,end_pos)
    % pause(2)

    prompt = "wait keyboard press\n";
    while input(prompt) == 1
    end

    
    % disp("------Adjusting Error------")
    % currentPos = vision_getPos();
    % error = goalPos - currentPos;
    % arm_adjustError(error)
    % pause(2)
    % 
    % 
    % disp("------Move to Place Pose------")
    % arm_Move2PlacePos()
    % pause(2)

    disp("------Releasing------") % complete
    gripper_release()
    % pause(2)
    
    % prompt = "wait keyboard press\n";
    % while input(prompt) == 1
    % end

    disp("------Task Completed------")

end

disp("------Complete Shape Sorting Task------")

% disp("------Back to Default Start Pose------")
% arm_Move2StartPose()
% pause(2)



