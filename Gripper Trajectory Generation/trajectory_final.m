%% DESCRIPTION
% this version is improved from final
% this follows the following logic
% > 135: back rotate, to 180 then back to goal angle
% < 135: forward rotate, to 90, to goal angle


% this version compenstae the error by adding offset to the rotate_start_pose
% we have basic movements from very beginning, which automatically selects 
% the slide with the least movement and
% shorted travel distance

% the slide part is simplified in "trajectoryV3"
% this is where everything is described using three-slides model
% and it automatically finds the least movement and shorted travel distance

% if we want to improve this so that it always try to find the solution
% where last move is move towards the centre, which could give us the
% desired pose regardless the friction status, look at "trajectoryV6".
% However, a problem might be
% that the last slide might be too small to fully remove the offset
% and this might be solved using the feature from this version where offset
% is fully removed through mathematical model



% start_pose is 2x1 array
% goal_pose is 3x1 array
start_pose = [0, 10];
goal_pose = [0, 10, 180];

motor_angle_trajectory = [];

goal_angle = goal_pose(3); 
if goal_angle > 180 && goal_angle <= 360
    goal_angle = goal_angle - 360; % rotate anticlockwise
elseif goal_angle > 360
    disp("Invalid angle input. Enter an angle that's within [-180, 360]")
end
goal_pose = [goal_pose(1:2), deg2rad(goal_angle-45)];


%% Parameters (unit: cm, degree)

% pad width, assume together for now
pad_thick = 0;
    
% object infor
L = 3.25; % square side length
square_diagonal = L * sqrt(2);

% finger width
W = 4; % finger_width = finger_width + 2 * rubber_width
FL = 20; % finger_length
W_high_friction = 2;

% base seperation
d = L+W; % base_seperate
base_left = [-d/2, 0];
base_right = [d/2, 0];
base_d = base_right(1) - base_left(1);

alpha1_LIMIT = deg2rad([20 160]); % NEED to find more reasonable figure
alpha2_LIMIT = deg2rad([20 160]);

low_fric = 0;
high_fric = 1;


% Plot start and end point
figure
scatter(start_pose(1), start_pose(2), 100, 'filled');
hold on;
scatter(goal_pose(1), goal_pose(2), 100, 'filled');
hold on;
   

%% Check if rotation needed
disp("Rotation Check")
fprintf("Rotate %d degree.\n",goal_angle)

rotate_small = mod(abs(goal_angle),90);
rotate_90 = (abs(goal_angle)-rotate_small)/90;

back2Mid_from_left = [0,pi/2,low_fric,high_fric];
back2Mid_from_right = [pi/2,0,high_fric,low_fric];


%% rotation between 90 to 180 or -90 to -180
if rotate_90 >= 1 && rotate_small > 0

    if goal_angle > 0 % Rotate from right to left
        disp("Rotate twice, clockwise")
        disp("Motor angle trajectory: ")
        
        if goal_angle > 135
            % rotate 90 clockwise, from R2L
            rotate1_start_pos = [start_pose(1:2),deg2rad(-90-45)];
            [motor_angle1, centre1] = get_L2R_Rotate_StartPos(square_diagonal, rotate1_start_pos, d, W, L, base_left, base_right, 0);
            % back to mid from right, get end pos
            rotate2_start_pos = [start_pose(1),start_pose(2)-L,deg2rad(-90-45)];
            % rotate 90 clockwise, from R2L
            [motor_angle2, centre2] = get_L2R_Rotate_StartPos(square_diagonal, rotate2_start_pos, d, W, L, base_left, base_right, 0);
            % back rotate
            goal_pos = [goal_pose(1:2),deg2rad(goal_angle-180-45)]; % rotate 90 clockwise, from L2R
            [motor_angle3, centre3] = get_L2R_Rotate_StartPos(square_diagonal, goal_pos, d, W, L, base_left, base_right, 1);
            % slide
            motor_angles = get_alpha_for_slide(centre3, centre2, base_left, base_right, base_d, FL, W_high_friction, L);


            motor_angle_trajectory = [motor_angle1(1,2),0,high_fric,high_fric; % rotate 90
                                    back2Mid_from_left; % back to mid
                                    motor_angle2(1,2),0,high_fric,high_fric; % rotate 90
                                    motor_angles; % slide
                                    motor_angle3(1,2),0,high_fric,low_fric; % slide
                                    0, motor_angle3(2,1),high_fric,high_fric % rotate goal-180
                                    ];
            disp(rad2deg(motor_angle_trajectory));

        elseif goal_angle <= 135
            
            rotate1_start_pos = [start_pose(1:2),deg2rad(-90-45)]; % rotate 90 anticlockwise, from R2L
            [motor_angle1, centre1] = get_L2R_Rotate_StartPos(square_diagonal, rotate1_start_pos, d, W, L, base_left, base_right, 0);
            goal_pos = [goal_pose(1:2),deg2rad(goal_angle-90-45)]; % rotate 90 clockwise, from L2R
            [motor_angle2, centre2] = get_R2L_Rotate_StartPos(square_diagonal, goal_pos, d, W, L, base_left, base_right, 1);
            motor_angles = get_alpha_for_slide(centre2, centre1, base_left, base_right, base_d, FL, W_high_friction, L);

            motor_angle_trajectory = [motor_angle1(1,2),0,high_fric,high_fric
                                    motor_angles;
                                    0,motor_angle2(1,1),low_fric,high_fric;
                                    motor_angle2(2,2),0,high_fric,high_fric
                                    ];
            disp(rad2deg(motor_angle_trajectory));

        end

    elseif goal_angle < 0
        disp("Rotate twice, anticlockwise")
        disp("Motor angle trajectory: ")
        
        if goal_angle < -135
            
            % rotate 90 anticlockwise, from R2L
            rotate1_start_pos = [start_pose(1:2),deg2rad(90-45)];
            [motor_angle1, centre1] = get_R2L_Rotate_StartPos(square_diagonal, rotate1_start_pos, d, W, L, base_left, base_right, 0);
            % back to mid from right, get end pos
            rotate2_start_pos = [start_pose(1),start_pose(2)-L,deg2rad(90-45)];
            % rotate 90 anticlockwise, from R2L
            [motor_angle2, centre2] = get_R2L_Rotate_StartPos(square_diagonal, rotate2_start_pos, d, W, L, base_left, base_right, 0);
            % back rotate
            goal_pos = [goal_pose(1:2),deg2rad(goal_angle+180-45)]; % rotate 90 clockwise, from L2R
            [motor_angle3, centre3] = get_R2L_Rotate_StartPos(square_diagonal, goal_pos, d, W, L, base_left, base_right, 1);
            % slide
            motor_angles = get_alpha_for_slide(centre3, centre2, base_left, base_right, base_d, FL, W_high_friction, L);


            motor_angle_trajectory = [motor_angle1(1,2),0,high_fric,high_fric; % rotate 90
                                    back2Mid_from_right; % back to mid
                                    motor_angle2(1,2),0,high_fric,high_fric; % rotate 90
                                    motor_angles; % slide
                                    motor_angle3(1,2),0,high_fric,low_fric; % slide
                                    0, motor_angle3(2,1),high_fric,high_fric % rotate goal-180
                                    ];
            disp(rad2deg(motor_angle_trajectory));



        elseif goal_angle >= -135

            rotate1_start_pos = [start_pose(1:2),deg2rad(90-45)]; % rotate 90 anticlockwise, from R2L
            [motor_angle1, centre1] = get_R2L_Rotate_StartPos(square_diagonal, rotate1_start_pos, d, W, L, base_left, base_right, 0);
            goal_pos = [goal_pose(1:2),deg2rad(goal_angle+90-45)]; % rotate 90 clockwise, from L2R
            [motor_angle2, centre2] = get_L2R_Rotate_StartPos(square_diagonal, goal_pos, d, W, L, base_left, base_right, 1);
            motor_angles = get_alpha_for_slide(centre2, centre1, base_left, base_right, base_d, FL, W_high_friction, L);
            
            motor_angle_trajectory = [0,motor_angle1(1), high_fric,high_fric;
                                    motor_angles;
                                    motor_angle2(1,2),0,high_fric,low_fric;
                                    0,motor_angle2(2,1),high_fric,high_fric
                                    ];     
            disp(rad2deg(motor_angle_trajectory));
        
        end

    else 
        disp("Error")
    end

%% without rotation or rotation between -90 to 90 && rotate 180
else 

    if goal_angle > 0 && goal_angle <= 90 % 0 to 90
        
        if goal_angle > 45
            
            % rotate 90 clockwise, from R2L
            rotate1_start_pos = [start_pose(1:2),deg2rad(-90-45)];
            [motor_angle1, centre1] = get_L2R_Rotate_StartPos(square_diagonal, rotate1_start_pos, d, W, L, base_left, base_right, 0);
            % back to mid from right, get end pos
            goal_pos = [goal_pose(1:2),deg2rad(goal_angle-90-45)]; % rotate 90 clockwise, from L2R
            [motor_angle2, centre2] = get_L2R_Rotate_StartPos(square_diagonal, goal_pos, d, W, L, base_left, base_right, 1);
            % slide
            motor_angles = get_alpha_for_slide(centre2, centre1, base_left, base_right, base_d, FL, W_high_friction, L);


            motor_angle_trajectory = [motor_angle1(1,2),0,high_fric,high_fric; % rotate 90
                                    motor_angles; % slide
                                    motor_angle2(1,2),0,high_fric,low_fric; % slide
                                    0, motor_angle2(2,1),high_fric,high_fric % rotate goal-180
                                    ];
            disp(rad2deg(motor_angle_trajectory));

        elseif goal_angle <= 45

            plotStartPos(start_pose, L, W, base_left, base_right);
            disp("SSSR, Clockwise Rotation") 
            [motor_angle1, centre1] = get_R2L_Rotate_StartPos(square_diagonal, goal_pose, d, W, L, base_left, base_right, 1);
            goal_pose = [centre1, goal_pose(3)];
            motor_angles = get_alpha_for_slide(goal_pose, start_pose, base_left, base_right, base_d, FL, W_high_friction, L);
    
            motor_angle_trajectory = [motor_angles;
                                    0,motor_angle1(1,1),low_fric,high_fric;
                                    motor_angle1(2,2),0,high_fric,high_fric
                                    ];
        end

        
    elseif goal_angle < 0 && goal_angle >= -90 % 0 to -90
        
        if goal_angle < -45

            % rotate 90 anticlockwise, from R2L
            rotate1_start_pos = [start_pose(1:2),deg2rad(90-45)];
            [motor_angle1, centre1] = get_R2L_Rotate_StartPos(square_diagonal, rotate1_start_pos, d, W, L, base_left, base_right, 0);
            % back rotate
            goal_pos = [goal_pose(1:2),deg2rad(goal_angle+90-45)]; % rotate 90 clockwise, from L2R
            [motor_angle2, centre2] = get_R2L_Rotate_StartPos(square_diagonal, goal_pos, d, W, L, base_left, base_right, 1);
            % slide
            motor_angles = get_alpha_for_slide(centre2, centre1, base_left, base_right, base_d, FL, W_high_friction, L);


            motor_angle_trajectory = [motor_angle1(1,2),0,high_fric,high_fric; % rotate 90
                                    motor_angles; % slide
                                    motor_angle2(1,2),0,high_fric,low_fric; % slide
                                    0, motor_angle2(2,1),high_fric,high_fric % rotate goal-180
                                    ];
            disp(rad2deg(motor_angle_trajectory));


        elseif goal_angle >= -45

            disp("SSSR, Anticlockwise Rotation")
            [motor_angle1, centre1] = get_L2R_Rotate_StartPos(square_diagonal, goal_pose, d, W, L, base_left, base_right, 1);
            goal_pose = [centre1, goal_pose(3)];
            % disp("Rotation Check Complete1")
            motor_angles = get_alpha_for_slide(goal_pose, start_pose, base_left, base_right, base_d, FL, W_high_friction, L);
    
            motor_angle_trajectory = [motor_angles;
                                    motor_angle1(1,2),0,high_fric,low_fric;
                                    0,motor_angle1(2,1),high_fric,high_fric
                                    ];  
            disp(rad2deg(motor_angle_trajectory))

        end

    elseif goal_angle == 0 % no rotation
        
        plotStartPos(start_pose, L, W, base_left, base_right);
        disp("SSS")
        disp("No rotation")
        motor_angles = get_alpha_for_slide(goal_pose, start_pose, base_left, base_right, base_d, FL, W_high_friction, L);
        
        motor_angle_trajectory = motor_angles;%slide motor angle
        disp(rad2deg(motor_angle_trajectory))


    elseif abs(goal_angle) == 180 % rotate 180

        % rotate 90 clockwise, from R2L
        rotate1_start_pos = [start_pose(1:2),deg2rad(-90-45)];
        [motor_angle1, centre1] = get_L2R_Rotate_StartPos(square_diagonal, rotate1_start_pos, d, W, L, base_left, base_right, 0);
        % back to mid from right, get end pos
        rotate2_start_pos = [start_pose(1),start_pose(2)-L,deg2rad(-90-45)];
        % rotate 90 clockwise, from R2L
        [motor_angle2, centre2] = get_L2R_Rotate_StartPos(square_diagonal, rotate2_start_pos, d, W, L, base_left, base_right, 0);
        % back rotate
        goal_pos = [goal_pose(1:2),deg2rad(goal_angle-180-45)]; % rotate 90 clockwise, from L2R
        [motor_angle3, centre3] = get_L2R_Rotate_StartPos(square_diagonal, goal_pos, d, W, L, base_left, base_right, 1);
        % slide
        motor_angles = get_alpha_for_slide(centre3, centre2, base_left, base_right, base_d, FL, W_high_friction, L);


        motor_angle_trajectory = [motor_angle1(1,2),0,high_fric,high_fric; % rotate 90
                                back2Mid_from_left; % back to mid
                                motor_angle2(1,2),0,high_fric,high_fric; % rotate 90
                                motor_angles; % slide
                                motor_angle3(1,2),0,high_fric,low_fric; % slide
                                0, motor_angle3(2,1),high_fric,high_fric % rotate goal-180
                                ];
        disp(rad2deg(motor_angle_trajectory));
        
    else 
        disp("Error")
        
    end

end 

% plotting setting
xlim([-10 10]);      % set x-axis limits
ylim([-5 10]);      % set y-axis limits
axis equal;
grid on;






%% Basic functions
function [] = plotArch (radius, origin, start_angle, end_angle)
    % Generate a set of angles to plot the circle
    theta = linspace(start_angle, end_angle, 100);
    
    % Calculate the x and y coordinates of the circle
    x = origin(1) + radius * cos(theta);
    y = origin(2) + radius * sin(theta);
    
    % Plot the circle
    plot(x, y, 'k', 'LineWidth', 2);
end

function angle = findalpha2(coord, base_right)
    x = coord(1);
    y = coord(2);
    angle = abs(atan( y / (base_right(1) - x) ));
    if x < base_right(1)
        angle = pi - angle; 
    end
end

function angle = findalpha1(coord, base_left)
    x = coord(1);
    y = coord(2);
    angle = atan( y / (base_left(1) - x) );
    if x > base_left(1)
        angle = pi + angle; 
    end
end

function [center, distance_to_base, angle_to_base] = getCentreInfor (square_four_points, base_right, base_left)
    % Define the coordinates of the four vertices of the square
    A = square_four_points(1,:); % Replace x1 and y1 with the x and y coordinates of vertex A
    B = square_four_points(2,:); % Replace x2 and y2 with the x and y coordinates of vertex B
    C = square_four_points(3,:); % Replace x3 and y3 with the x and y coordinates of vertex C
    D = square_four_points(4,:); % Replace x4 and y4 with the x and y coordinates of vertex D
    
    % Compute the midpoint of each pair of opposite vertices
    M_AB = (A + B) / 2;
    M_BC = (B + C) / 2;
    M_CD = (C + D) / 2;
    M_DA = (D + A) / 2;
    
    % Calculate the average of the four center points
    center = (M_AB + M_BC + M_CD + M_DA) / 4;
    
    centre_to_baseR = sqrt((base_right(1)-center(1))^2 + (center(2))^2);
    centre_to_baseL = sqrt((base_left(1)-center(1))^2 + (center(2))^2);

    angle_to_baseR = atan( center(2) / (base_right(1)-center(1)) );
    angle_to_baseL = atan( center(2) / (base_left(1)-center(1)) );

    distance_to_base = [centre_to_baseL; centre_to_baseR];
    angle_to_base = [angle_to_baseL, angle_to_baseR];
end

function [] = plotStartPos(start_pose, L, W, base_left, base_right)
    %start pos process
    % a_start = [start_pose(1)-square_diagonal/2*cos(start_pose(3)) start_pose(2)+square_diagonal/2*sin(goal_pose(3))];
    % b_start = [start_pose(1)+square_diagonal/2*cos(start_pose(3)) start_pose(2)-square_diagonal/2*sin(goal_pose(3))];

    a_start = [start_pose(1)-L/2,start_pose(2)-L/2];
    b_start = [start_pose(1)+L/2,start_pose(2)+L/2]; 

    alpha1_start = findalpha1(start_pose,base_left);
    alpha2_start = findalpha2(start_pose,base_right);
    
    b_length_start = sqrt((W/2)^2 + b_start(2)^2);
    a_length_start = sqrt((W/2)^2 + a_start(2)^2);
    
    left_offset = atan((W/2)/a_start(2));
    right_offset = atan((W/2)/b_start(2));

    motor_angle_start = [alpha1_start-left_offset, alpha2_start-right_offset];
    
    %plot start pos square
    
    square_coord_start = [a_start;
        b_start(1),a_start(2);
        b_start;
        a_start(1), b_start(2);
        a_start;
        b_start;
        ];
    
    plot(square_coord_start(:,1), square_coord_start(:,2), 'g', 'LineWidth', 0.5);
    hold on;


    Left_touch_line = [base_left; a_start];
    plot(Left_touch_line(:,1), Left_touch_line(:,2),'r', 'LineWidth', 0.5);
    hold on;

    Right_touch_line = [base_right; b_start];
    plot(Right_touch_line(:,1), Right_touch_line(:,2),'r', 'LineWidth', 0.5)
    hold on;

    [centre_start, distance_to_base_start, angle_to_base_start] = getCentreInfor(square_coord_start(1:4,:), base_right, base_left);
    hold on;

    triangle_start = [
        b_start;
        centre_start;
        a_start(1), b_start(2)
        ];
    plot(triangle_start(:,1), triangle_start(:,2), 'k', 'LineWidth', 0.5);
    hold on;
end


%% rotate from positive x-axis to any other place that's on the left side of the finger
function [motor_angle1, centre1] = get_R2L_Rotate_StartPos(square_diagonal, goal_pose, d, W, L, base_left, base_right, last_move_index)
    
    a_goal = [goal_pose(1)-square_diagonal/2*cos(goal_pose(3)) goal_pose(2)+square_diagonal/2*sin(goal_pose(3))];
    b_goal = [goal_pose(1)+square_diagonal/2*cos(goal_pose(3)) goal_pose(2)-square_diagonal/2*sin(goal_pose(3))];
    
    % length and angle of the line connect actuator centre and touch point
    a_length_goal = sqrt((a_goal(1)+d/2)^2+a_goal(2)^2);
    b_length_goal = sqrt((b_goal(1)-d/2)^2+b_goal(2)^2);

    alpha1_goal = findalpha1(a_goal, base_left);
    alpha2_goal = findalpha2(b_goal, base_right);
      
    % offset due to thickness of gripper
    left_angle_offset = asin( (W/2) /a_length_goal);
    right_angle_offset = asin( (W/2) /b_length_goal);    
    
    theta1 = asin((W/2)/b_length_goal);
    theta2 = theta1 + pi/4;
    l1 = sqrt( (square_diagonal)^2 + (b_length_goal)^2 - (cos(theta2)*2*square_diagonal*b_length_goal) );
    theta3 = acos( ((d)^2 + (a_length_goal)^2 - (l1)^2) / (2*d*a_length_goal));
    alpha1_rotate = pi - theta3;
    
    theta4 = acos( ((l1)^2+(square_diagonal)^2-(b_length_goal)^2) / (2*l1*square_diagonal) );
    theta5 = acos( ((a_length_goal)^2 + (l1)^2 - (d)^2) / (2*a_length_goal*l1));
    alpha2_rotate = pi - (2*pi - theta3 - theta4 - theta5 - pi/4) + right_angle_offset;

    % motor angles
    left_motor_angle_rotate = alpha1_rotate - left_angle_offset;
    left_motor_angle_goal = alpha1_goal - left_angle_offset;
    right_motor_angle_rotate = pi - (2*pi - theta3 - theta4 - theta5 - pi/4);
    right_motor_angle_goal = alpha2_goal - right_angle_offset;


    %a1 = [base_left(1) + a_length*cos(theta3), base_left(2) + a_length*sin(theta3)];
    a1 = [base_left(1) - a_length_goal*cos(alpha1_rotate), base_left(2) + a_length_goal*sin(alpha1_rotate)];
    b1 = [base_right(1) + b_length_goal*cos(alpha2_rotate), base_right(2) + b_length_goal*sin(alpha2_rotate)];
    b_low = [b1(1)-L*cos(right_motor_angle_rotate), b1(2)-L*sin(right_motor_angle_rotate)];
    


    
    if last_move_index == 1
        % add offset
        b_low_length = sqrt((b_low(1)-base_right(1))^2 + b_low(2)^2);
        theta6 = left_angle_offset+pi/2;
        l2 = sqrt( (a_length_goal)^2 + (L)^2 - (cos(theta6)*2*a_length_goal*L) );
        theta7 = asin( (L/l2) * sin(theta6) );
        theta8 = acos( ((d)^2 + (l2)^2 - (b_low_length)^2) / (2*d*l2));
        left_motor_actual = pi-theta7-theta8-left_angle_offset; % left motor angle when in undesired pose
        
        motor_angle1 = [left_motor_actual, right_motor_angle_rotate;
                        left_motor_angle_goal, right_motor_angle_goal
                        ];

        disp("left_motor_actual")
        % disp(a_length_goal)
        % % disp(l2)
        % disp(rad2deg(left_angle_offset))
        % disp(rad2deg(left_motor_angle1))
        % disp(rad2deg(left_motor_actual));

    elseif last_move_index == 0
        
        motor_angle1 = [left_motor_angle_rotate, right_motor_angle_rotate;
                        left_motor_angle_goal, right_motor_angle_goal
                        ];
    else
        disp("last move index error for get_R2L_Rotate_StartPos")
    end
    
    





    % diagrams
    % Square
    square_coord_rotate = [a1;
    b_low;
    b1;
    b1(1)-L*sin(right_motor_angle_rotate), b1(2)+L*cos(right_motor_angle_rotate);
    a1;
    b1;
    ];
    
    square_coord_goal = [a_goal; 
    a_goal(1) + L*sin((pi/4)-goal_pose(3)), a_goal(2) - L*cos((pi/4)-goal_pose(3));
    b_goal;
    b_goal(1) - L*sin((pi/4)-goal_pose(3)), b_goal(2) + L*cos((pi/4)-goal_pose(3));
    a_goal;
    b_goal;
    ];
    
    plot(square_coord_rotate(:,1), square_coord_rotate(:,2), 'b-', 'LineWidth', 0.5);
    hold on;
    plot(square_coord_goal(:,1), square_coord_goal(:,2), 'b-', 'LineWidth', 0.5);
    hold on;

    % touch lines
    Left_touch_line = [base_left; a1];
    plot(Left_touch_line(:,1), Left_touch_line(:,2),'r', 'LineWidth', 0.5);
    hold on;

    Right_touch_line = [base_right; b1];
    plot(Right_touch_line(:,1), Right_touch_line(:,2),'r', 'LineWidth', 0.5)
    hold on;
    
    Left_touch_line = [base_left;
    base_left(1) - a_length_goal*cos(alpha1_goal), base_left(2) + a_length_goal*sin(alpha1_goal)];
    plot(Left_touch_line(:,1), Left_touch_line(:,2),'r', 'LineWidth', 0.5)
    hold on;
    
    Right_touch_line = [base_right;
        base_right(1) + b_length_goal*cos(alpha2_goal), base_right(2) + b_length_goal*sin(alpha2_goal)];
    plot(Right_touch_line(:,1), Right_touch_line(:,2),'r', 'LineWidth', 0.5)
    hold on;

    % get centre
    [centre1, distance_to_base1, angle_to_base1] = getCentreInfor(square_coord_rotate(1:4,:), base_right, base_left);
    scatter(centre1(1), centre1(2), 100, 'filled');
    hold on;

    [centre_goal, distance_to_base_goal, angle_to_base_goal] = getCentreInfor(square_coord_goal(1:4,:), base_right, base_left);
    scatter(centre1(1), centre1(2), 100, 'filled');
    hold on;
    
    % direction 
    triangle = [b1;
        centre1;
        b1(1)-L*sin(right_motor_angle_rotate), b1(2)+L*cos(right_motor_angle_rotate)
        ];
    plot(triangle(:,1), triangle(:,2), 'k', 'LineWidth', 0.5);
    hold on;

    triangle_goal = [b_goal;
        centre_goal
        b_goal(1) - L*sin((pi/4)-goal_pose(3)), b_goal(2) + L*cos((pi/4)-goal_pose(3));
        ];
    plot(triangle_goal(:,1), triangle_goal(:,2), 'k', 'LineWidth', 0.5);
    hold on;
end

%% rotate from negative x-axis to any other place that's on the right side of the finger
function [motor_angle1, centre1] = get_L2R_Rotate_StartPos(square_diagonal, goal_pose, d, W, L, base_left, base_right, last_move_index)
    angle = -(goal_pose(3)+pi/4) - pi/4;

    a_goal = [goal_pose(1)-square_diagonal/2*cos(angle) goal_pose(2)-square_diagonal/2*sin(angle)];
    b_goal = [goal_pose(1)+square_diagonal/2*cos(angle) goal_pose(2)+square_diagonal/2*sin(angle)];

    % length and angle of the line connect actuator centre and touch point
    a_length_goal = sqrt((a_goal(1)-(-d/2))^2+a_goal(2)^2);
    b_length_goal = sqrt((b_goal(1)-d/2)^2+b_goal(2)^2);
      
    % offset due to thickness of gripper
    left_angle_offset = asin( (W/2) /a_length_goal);
    right_angle_offset = asin( (W/2) /b_length_goal);   
    
    alpha1_goal = findalpha1(a_goal, base_left);
    alpha2_goal = findalpha2(b_goal, base_right);
    

    % Calculate two alphas and motor angle
    theta1 = asin((W/2)/a_length_goal);
    theta2 = theta1 + pi/4;
    l1 = sqrt( (square_diagonal)^2 + (a_length_goal)^2 - (cos(theta2)*2*square_diagonal*a_length_goal) );
    alpha2_rotate = pi - acos( ((d)^2 + (b_length_goal)^2 - (l1)^2) / (2*d*b_length_goal));

    theta4 = acos( ( (l1)^2+(a_length_goal)^2-(square_diagonal)^2 ) / (2*l1*a_length_goal) );
    theta5 = acos( ((d)^2 + (l1)^2 - (b_length_goal)^2) / (2*d*l1));
    alpha1_rotate = pi - theta4 - theta5;

    % motor angle
    right_motor_angle_rotate = alpha2_rotate - right_angle_offset;
    right_motor_angle_goal = alpha2_goal - right_angle_offset;
    left_motor_angle_rotate = alpha1_rotate - left_angle_offset;
    left_motor_angle_goal = alpha1_goal - left_angle_offset;
    
    
    a1 = [base_left(1) - a_length_goal*cos(alpha1_rotate), a_length_goal*sin(alpha1_rotate)];
    b1 = [base_right(1) - b_length_goal*cos(pi-alpha2_rotate), b_length_goal*sin(pi-alpha2_rotate)];
    a_low = [a1(1)+L*cos(left_motor_angle_rotate), a1(2)-L*sin(left_motor_angle_rotate)];
    % b_low1 = [b1(1)-L*cos(right_motor_angle), b1(2)-L*sin(right_motor_angle)];

    if last_move_index == 1
        % add offset
        a_low_length = sqrt((a_low(1)-base_left(1))^2 + a_low(2)^2);
        theta6 = left_angle_offset+pi/2;
        l2 = sqrt( (a_length_goal)^2 + (L)^2 - (cos(theta6)*2*a_length_goal*L) );
        theta7 = asin( (L/l2) * sin(theta6) );
        theta8 = acos( ((d)^2 + (l2)^2 - (a_low_length)^2) / (2*d*l2));
        left_motor_actual = pi-theta7-theta8-left_angle_offset; % left motor angle when in undesired pose
        
        motor_angle1 = [left_motor_actual, right_motor_angle_rotate;
                        left_motor_angle_goal, right_motor_angle_goal
                        ];

        disp("left_motor_actual")
        
    elseif last_move_index == 0
        motor_angle1 = [left_motor_angle_rotate, right_motor_angle_rotate;
                    left_motor_angle_goal, right_motor_angle_goal];
    else
        disp("last move index error for get_L2R_Rotate_StartPos")
    end

    % touch lines
    Left_touch_line = [base_left; a1];
    plot(Left_touch_line(:,1), Left_touch_line(:,2),'r', 'LineWidth', 0.5);
    hold on;

    Right_touch_line = [base_right; b1];
    plot(Right_touch_line(:,1), Right_touch_line(:,2),'r', 'LineWidth', 0.5)
    hold on;
    
    Left_touch_line = [base_left;
    base_left(1) - a_length_goal*cos(alpha1_goal), base_left(2) + a_length_goal*sin(alpha1_goal)];
    plot(Left_touch_line(:,1), Left_touch_line(:,2),'r', 'LineWidth', 0.5)
    hold on;

    Right_touch_line = [base_right;
        base_right(1) + b_length_goal*cos(alpha2_goal), base_right(2) + b_length_goal*sin(alpha2_goal)];
    plot(Right_touch_line(:,1), Right_touch_line(:,2),'r', 'LineWidth', 0.5)
    hold on;

    % Square
    square_coord_rotate = [a1;
        a1(1)+L*cos(left_motor_angle_rotate), a1(2)-L*sin(left_motor_angle_rotate);
        b1;
        b1(1)-L*cos(left_motor_angle_rotate), b1(2)+L*sin(left_motor_angle_rotate);
        a1;
        b1;
        ];
    
    square_coord_goal = [a_goal; 
        a_goal(1) + L*cos((pi/4)-angle), a_goal(2) - L*sin((pi/4)-angle);
        b_goal;
        b_goal(1) - L*cos((pi/4)-angle), b_goal(2) + L*sin((pi/4)-angle);
        a_goal;
        b_goal;
    ];
    
    plot(square_coord_rotate(:,1), square_coord_rotate(:,2), 'b-', 'LineWidth', 0.5);
    hold on;
    plot(square_coord_goal(:,1), square_coord_goal(:,2), 'b-', 'LineWidth', 0.5);
    hold on;

    % get centre
    [centre1, distance_to_base1, angle_to_base1] = getCentreInfor(square_coord_rotate(1:4,:), base_right, base_left);
    scatter(centre1(1), centre1(2), 100, 'filled');
    hold on;

    [centre_goal, distance_to_base1, angle_to_base1] = getCentreInfor(square_coord_goal(1:4,:), base_right, base_left);
    scatter(centre_goal(1), centre_goal(2), 100, 'filled');
    hold on;
    
    % direction 
    triangle = [
        b1(1)-L*cos(left_motor_angle_rotate), b1(2)+L*sin(left_motor_angle_rotate);
        centre1;
        a1;
        ];
    plot(triangle(:,1), triangle(:,2), 'r', 'LineWidth', 0.5);
    hold on;

    triangle_goal = [
        b_goal(1) - L*cos((pi/4)-angle), b_goal(2) + L*sin((pi/4)-angle);
        centre_goal;
        a_goal;
        ];
    plot(triangle_goal(:,1), triangle_goal(:,2), 'r', 'LineWidth', 0.5);
    hold on;

end


%% Slide
function [motor_angles] = get_alpha_for_slide(goal_pose, start_pose, base_left, base_right, base_d, FL, W_high_friction, L)
    
    r_start_from_left = sqrt((start_pose(1)-base_left(1))^2 + start_pose(2)^2);
    r_start_from_right = sqrt((start_pose(1)-base_right(1))^2 + start_pose(2)^2);

    r_goal_from_left = sqrt((goal_pose(1)-base_left(1))^2 + goal_pose(2)^2);
    r_goal_from_right = sqrt((goal_pose(1)-base_right(1))^2 + goal_pose(2)^2);

    low_fric = 0;
    high_fric = 1;


    if r_goal_from_right > FL || r_goal_from_left > FL
        disp(r_goal_from_right)
        disp(r_goal_from_left)
        disp("Invalid Coordinate. Unreachable. Check.")
    
    elseif goal_pose(1:2) == start_pose()
        disp("no movement")
    



% one slide ------------------
% centre_to_baseR = sqrt((base_right(1)-center(1))^2 + (center(2))^2);
% centre_to_baseL = sqrt((base_left(1)-center(1))^2 + (center(2))^2);

    elseif r_goal_from_left == r_start_from_left
        disp("One Slide")
        disp("Control Mode: P,T")
        disp("High Friction Sequence: L")
    
        alpha1_start = findalpha1(start_pose,base_left);
        alpha1_end = findalpha1(goal_pose,base_left);
    
        plotArch (r_start_from_left,base_left,pi-alpha1_start,pi-alpha1_end);
        hold on;
        
        % calculate motor angle
        centre_to_baseL = sqrt((base_left(1)-start_pose(1))^2 + (start_pose(2))^2);
        d_angle = asin((L/2+W_high_friction/2)/(centre_to_baseL/sin(pi/2)));
        left_motor_angle_start = alpha1_start-d_angle;
        left_motor_angle_end = alpha1_end-d_angle;
        
        motor_angles = [0 left_motor_angle_start low_fric high_fric;
                        0 left_motor_angle_end low_fric high_fric]; 

    elseif r_goal_from_right == r_start_from_right
        disp("One Slide")
        disp("Control Mode: T,P")
        disp("High Friction Sequence: R")
    
        alpha2_start = findalpha2(start_pose,base_right);
        alpha2_end = findalpha2(goal_pose,base_right);
    
        plotArch (r_start_from_right,base_right,alpha2_end,alpha2_start);
        hold on;

        % calculate motor angle
        centre_to_baseR = sqrt((base_right(1)-start_pose(1))^2 + (start_pose(2))^2);
        d_angle = asin((L/2+W_high_friction/2)/(centre_to_baseR/sin(pi/2)));
        right_motor_angle_start = alpha2_start-d_angle;
        right_motor_angle_end = alpha2_end-d_angle;

        motor_angles = [right_motor_angle_start 0 high_fric low_fric;
                        right_motor_angle_end 0 high_fric low_fric];
        

% two slide ------------------

    elseif r_goal_from_right < (base_d + r_start_from_left) || r_goal_from_left < (base_d + r_start_from_right)
        disp("Two Slides")

% can only to Left then Right
        if r_goal_from_right < (base_d + r_start_from_left) && r_goal_from_left > (base_d + r_start_from_right)
            
            [alpha1,alpha2, angle_moved_LR] = twoSlides_LR(start_pose,goal_pose,base_left,base_right,base_d,r_start_from_left,r_goal_from_right);
            
            plotArch (r_start_from_left,base_left,pi-alpha1(2),pi-alpha1(1));
            hold on;
            plotArch (r_goal_from_right,base_right,alpha2(2),alpha2(1));
            hold on;
            
            disp("to Left then Right")

            % red is left_motor, white is right_motor
            % [XM(white), XM(red), white-DL(index=1), red-DL(index=2)]

            centre_to_baseL = sqrt((base_left(1)-start_pose(1))^2 + (start_pose(2))^2);
            d_angle = asin((L/2+W_high_friction/2)/(centre_to_baseL/sin(pi/2)));
            left_motor_angle_start = alpha1(1)-d_angle;
            left_motor_angle_end = alpha1(2)-d_angle;
            
            centre_to_baseR = sqrt((base_right(1)-goal_pose(1))^2 + (goal_pose(2))^2);
            d_angle = asin((L/2+W_high_friction/2)/(centre_to_baseR/sin(pi/2)));
            right_motor_angle_end = alpha2(2)-d_angle;
    
            
            motor_angles = [0 left_motor_angle_end low_fric high_fric;
                                right_motor_angle_end 0 high_fric low_fric;
                                right_motor_angle_end-deg2rad(30) 0 high_fric low_fric];


% can only to right then left
        elseif r_goal_from_right > (base_d + r_start_from_left) && r_goal_from_left < (base_d + r_start_from_right)
            
            [alpha1,alpha2, angle_moved_RL] = twoSlides_RL(start_pose,goal_pose,base_left,base_right,base_d,r_start_from_right,r_goal_from_left);
            
            plotArch (r_goal_from_left,base_left,pi-alpha1(2),pi-alpha1(1));
            hold on;
            plotArch (r_start_from_right,base_right,alpha2(2),alpha2(1));
            hold on;

            disp("to Right then Left")

            centre_to_baseR = sqrt((base_right(1)-start_pose(1))^2 + (start_pose(2))^2);
            d_angle = asin((L/2+W_high_friction/2)/(centre_to_baseR/sin(pi/2)));
            right_motor_angle_start = alpha2(1)-d_angle;
            right_motor_angle_end = alpha2(2)-d_angle;
            
            centre_to_baseL = sqrt((base_left(1)-goal_pose(1))^2 + (goal_pose(2))^2);
            d_angle = asin((L/2+W_high_friction/2)/(centre_to_baseL/sin(pi/2)));
            left_motor_angle_end = alpha1(2)-d_angle;
    
            
            motor_angles = [right_motor_angle_end 0 high_fric low_fric;
                            0 left_motor_angle_end low_fric high_fric;
                            0 left_motor_angle_end-deg2rad(30) low_fric high_fric];


% can either way, find the best way
        else
            [alpha1_LR,alpha2_LR, angle_moved_LR] = twoSlides_LR(start_pose,goal_pose,base_left,base_right,base_d,r_start_from_left,r_goal_from_right);
            [alpha1_RL,alpha2_RL, angle_moved_RL] = twoSlides_RL(start_pose,goal_pose,base_left,base_right,base_d,r_start_from_right,r_goal_from_left);
            
            disp("either")

            % if goal_pose(1) < 0
            % 
            %     alpha1 = alpha1_LR;
            %     alpha2 = alpha2_LR;
            % 
            %     plotArch (r_start_from_left,base_left,pi-alpha1(2),pi-alpha1(1));
            %     hold on;
            %     plotArch (r_goal_from_right,base_right,alpha2(2),alpha2(1));
            %     hold on;
            % 
            %     centre_to_baseL = sqrt((base_left(1)-start_pose(1))^2 + (start_pose(2))^2);
            %     d_angle = asin((L/2+W_high_friction/2)/(centre_to_baseL/sin(pi/2)));
            %     % left_motor_angle_start = alpha1(1)-d_angle;
            %     left_motor_angle_end = alpha1(2)-d_angle;
            % 
            %     centre_to_baseR = sqrt((base_right(1)-goal_pose(1))^2 + (goal_pose(2))^2);
            %     d_angle = asin((L/2+W_high_friction/2)/(centre_to_baseR/sin(pi/2)));
            %     right_motor_angle_end = alpha2(2)-d_angle;
            % 
            %     disp("rotate round left then right")
            % 
            %     motor_angles = [0 left_motor_angle_end low_fric high_fric;
            %                     right_motor_angle_end 0 high_fric low_fric;
            %                     right_motor_angle_end-deg2rad(30) 0 high_fric low_fric];
            % 
            % else % goal_pose(1) >= 0

                alpha1 = alpha1_RL;
                alpha2 = alpha2_RL;

                plotArch (r_goal_from_left,base_left,pi-alpha1(2),pi-alpha1(1));
                hold on;
                plotArch (r_start_from_right,base_right,alpha2(2),alpha2(1));
                hold on;

                centre_to_baseR = sqrt((base_right(1)-start_pose(1))^2 + (start_pose(2))^2);
                d_angle = asin((L/2+W_high_friction/2)/(centre_to_baseR/sin(pi/2)));
                right_motor_angle_end = alpha2(2)-d_angle;
                right_motor_angle_end1 = alpha2(3)-d_angle;

                centre_to_baseL = sqrt((base_left(1)-goal_pose(1))^2 + (goal_pose(2))^2);
                d_angle = asin((L/2+W_high_friction/2)/(centre_to_baseL/sin(pi/2)));
                left_motor_angle_end = alpha1(2)-d_angle;
                left_motor_angle_end1 = alpha1(1)-d_angle;
                

                disp("rotate around right then left")

                motor_angles = [0 left_motor_angle_end1 high_fric low_fric];
                            % right_motor_angle_end1 0 low_fric high_fric];

                % disp(rad2deg(motor_angles))
            % end

            

        end
    
 

% three slide ------------------

    elseif r_goal_from_right >= (base_d + r_start_from_left)
        % if three slides AND rotate_start_pos is on the negative x-axis
        % choose this way of slide
    
        r_goal = r_goal_from_left;

        %alpha1 = findShortRoute(r_start, r_goal, base_d, base_left,start_pose,goal_pose);
        
        % a = 1; % need to know how to obtain the best a value
        a = find_3_slide_SR(start_pose,goal_pose,base_left,base_d,r_start_from_left,r_goal_from_left); % min
        
        b = sqrt(r_start_from_left^2-a^2);
        r_mid = sqrt( (a+base_d)^2 + (b)^2 );

        alpha1_start = findalpha1(start_pose,base_left);
        alpha2_start = findalpha2(start_pose,base_right);

        alpha1_mid1 = pi - acos( (r_start_from_left^2 + base_d^2 - r_mid^2) / (2*r_start_from_left*base_d) );
        alpha2_mid1 = pi - acos( (-r_start_from_left^2 + base_d^2 + r_mid^2) / (2*r_mid*base_d) );

        alpha1_mid2 = pi - acos( (-r_mid^2 + base_d^2 + r_goal^2) / (2*r_goal*base_d) );
        alpha2_mid2 = pi - acos( (r_mid^2 + base_d^2 - r_goal^2) / (2*r_mid*base_d) );

        alpha1_end = findalpha1(goal_pose,base_left);
        alpha2_end = findalpha2(goal_pose,base_right);

        plotArch (r_start_from_left,base_left,pi-alpha1_start,pi-alpha1_mid1);
        hold on;
        plotArch (r_mid,base_right,alpha2_mid2,alpha2_mid1);
        hold on;
        plotArch (r_goal,base_left,pi-alpha1_end,pi-alpha1_mid2);
        hold on;

        centre_to_baseR = sqrt((base_right(1)-start_pose(1))^2 + (start_pose(2))^2);
        d_angle = asin((L/2+W_high_friction/2)/(centre_to_baseR/sin(pi/2)));
        right_motor_angle_end = alpha2(2)-d_angle;

        centre_to_baseL = sqrt((base_left(1)-goal_pose(1))^2 + (goal_pose(2))^2);
        d_angle = asin((L/2+W_high_friction/2)/(centre_to_baseL/sin(pi/2)));
        left_motor_angle_end = alpha1(2)-d_angle;

        motor_angles = [right_motor_angle_end 0 high_fric low_fric;
                        0 left_motor_angle_end high_fric low_fric];

    else 
        disp("Slide Error")
        disp(r_goal_from_right)

    end
end

function [alpha1,alpha2, angle_moved] = twoSlides_LR(start_pose,goal_pose,base_left,base_right,base_d,r_start_from_left,r_goal_from_right)
    r_goal = r_goal_from_right;
    
    alpha1_start = findalpha1(start_pose,base_left);
    alpha2_end = findalpha2(goal_pose,base_right);

    alpha1_mid = pi - acos( (-r_goal^2 + r_start_from_left^2 + base_d^2) / (2*r_start_from_left*base_d) );
    alpha2_mid = pi - acos((r_goal^2 + base_d^2 - r_start_from_left^2) / (2*base_d*r_goal));

    alpha1 = [alpha1_start,alpha1_mid];
    alpha2 = [alpha2_mid,alpha2_end];
    
    % disp(alpha1)
    % disp(alpha2)
    disp("Wrong trajectory")
    disp(alpha1)
    disp(alpha2)

    angle_moved = abs(alpha1(1) - alpha1(2)) + abs(alpha2(1) - alpha2(2));
    % disp(angle_moved)

end

function [alpha1,alpha2, angle_moved] = twoSlides_RL(start_pose,goal_pose,base_left,base_right,base_d,r_start_from_right,r_goal_from_left)
    r_goal = r_goal_from_left;
    

    alpha2_start = findalpha2(start_pose,base_right);
    alpha1_start = findalpha1(start_pose,base_left);

    alpha2_mid = pi - acos( (-r_goal^2 + r_start_from_right^2 + base_d^2) / (2*r_start_from_right*base_d) );
    alpha1_mid = pi - acos((r_goal^2 + base_d^2 - r_start_from_right^2) / (2*base_d*r_goal));

    alpha1_end = findalpha1(goal_pose,base_left);
    alpha2_end = findalpha2(goal_pose,base_right);

    alpha1 = [alpha1_mid,alpha1_end, alpha1_start];
    alpha2 = [alpha2_start,alpha2_mid,alpha2_end];
    
    % disp("jkdbaskdkjasbdkjbk")
    % disp(alpha1)
    % disp(alpha2)

    angle_moved = abs(alpha1(1) - alpha1(2)) + abs(alpha2(1) - alpha2(2));
    % disp(angle_moved)
end

function [alpha1,alpha2,angle_moved] = threeSlides_LR(start_pose,goal_pose,base_left,base_d,r_start_from_left,r_goal_from_left, a)
    r_goal = r_goal_from_left;
    
    %alpha1 = findShortRoute(r_start, r_goal, base_d, base_left,start_pose,goal_pose);

    % a = 1; % need to know how to obtain the best a value
    b = sqrt(r_start_from_left^2-a^2);
    r_mid = sqrt( (a+base_d)^2 + (b)^2 );

    alpha1_start = findalpha1(start_pose,base_left);
    alpha1_end = findalpha1(goal_pose,base_left);

    alpha1_mid1 = pi - acos( (r_start_from_left^2 + base_d^2 - r_mid^2) / (2*r_start_from_left*base_d) );
    alpha2_mid1 = pi - acos( (-r_start_from_left^2 + base_d^2 + r_mid^2) / (2*r_mid*base_d) );

    alpha1_mid2 = pi - acos( (-r_mid^2 + base_d^2 + r_goal^2) / (2*r_goal*base_d) );
    alpha2_mid2 = pi - acos( (r_mid^2 + base_d^2 - r_goal^2) / (2*r_mid*base_d) );

    alpha1 = [alpha1_start,alpha1_mid1,alpha1_mid2,alpha1_end];
    alpha2 = [alpha2_mid1,alpha2_mid2];

    angle_moved = abs(alpha1_start - alpha1_mid1) + abs(alpha2_mid2 - alpha2_mid1) + abs(alpha1_mid2 - alpha1_end);
end

function [a] = find_3_slide_SR(start_pose,goal_pose,base_left,base_d,r_start_from_left,r_goal_from_left)
    i = 0;
    angle_moved = [];
    for a = -r_start_from_left:0.1:r_start_from_left
        i = i+1;
        [~, ~, angle_moved(i)] = threeSlides_LR(start_pose,goal_pose,base_left,base_d,r_start_from_left,r_goal_from_left, a);    
    end

    [minValue, minIndex] = min(angle_moved);
    a = -r_start_from_left + (minIndex-1)*0.1;
end


% function [isok] = checkLimit(alpha1, alpha2, alpha1_LIMIT, alpha2_LIMIT)
%     if alpha1 > alpha1_LIMIT(1) || alpha2 > alpha2_LIMIT(1) || alpha1 < alpha1_LIMIT(2) || alpha2 < alpha2_LIMIT(2)
%         isok = 0;
%     end   
%     isok = 1;
% end

% function [] = getoffset()
% 
% end

%% Optimisation - gradient decent
% function [] = gradient_decent(theta)
%     while (abs(gradient)<= 1^(-5))
% 
%     end
% end

