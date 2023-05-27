%% clean
clear all

%% parameters
% unit: cm, degree

% pad width, assume together for now
pad_thick = 0;

% start pose
start_pose = [0, 3];

% goal pose, assume x = 0 all the time
goal_y = 8.5;
goal_angle = 0; %负值由右往左转得到, 正值由左往右转得到 -45 means no rotation
% this is the angle between x-axis and diagonal line, negative is
% anticlockwise, positive is clockwise
goal_pose = [0, goal_y, goal_angle*pi/180];

% object infor
L = 2; % square side length
square_diagonal = L * sqrt(2);

% finger width
W = 2; % finger_width = finger_width + 2 * rubber_width

% base seperation
d = L+W; % base_seperate
base_left = [-d/2, 0];
base_right = [d/2, 0];
% push against left finger, left finger position control
% right finer torque control


%% plot setting



%% start pos process

a_start = [-L/2,start_pose(2)-L/2];
b_start = [L/2,start_pose(2)+L/2]; 

motor_angle_start = [pi/2, pi/2];

b_length_start = sqrt((W/2)^2 + b_start(2)^2);
a_length_start = sqrt((W/2)^2 + a_start(2)^2);

left_offset = atan((W/2)/a_start(2));
right_offset = atan((W/2)/b_start(2));

left_angle_start = pi/2+left_offset;
right_angle_start = pi/2+right_offset;

%% plot start pos square

square_coord_start = [a_start;
    b_start(1),a_start(2);
    b_start;
    a_start(1), b_start(2);
    a_start;
    b_start;
    ];

plot(square_coord_start(:,1), square_coord_start(:,2), 'g', 'LineWidth', 0.5);
hold on;

Left_touch_line = [base_left;
    base_left(1) + a_length_start*cos(pi-left_angle_start), base_left(2) + a_length_start*sin(pi-left_angle_start)];

plot(Left_touch_line(:,1), Left_touch_line(:,2),'r', 'LineWidth', 0.5)
hold on;

Right_touch_line = [base_right;
    base_right(1) - b_length_start*cos(pi-right_angle_start), base_right(2) + b_length_start*sin(pi-right_angle_start)];

plot(Right_touch_line(:,1), Right_touch_line(:,2),'r', 'LineWidth', 0.5)
hold on;


%% calculate goal pos's motor angle
% push against left finger, left finger position control
% right finer torque control

% a is the left touch point of the square, b is the right touch point of
% the sqaure

a_goal = [-square_diagonal/2*cos(goal_pose(3)) goal_pose(2)+square_diagonal/2*sin(goal_pose(3))];
b_goal = [square_diagonal/2*cos(goal_pose(3)) goal_pose(2)-square_diagonal/2*sin(goal_pose(3))];

% length and angle of the line connect actuator centre and touch point
a_length = sqrt((a_goal(1)+d/2)^2+a_goal(2)^2);
b_length = sqrt((b_goal(1)-d/2)^2+b_goal(2)^2);

left_angle = acos( (-a_goal(1)-d/2) / (a_length) );
right_angle = acos( (b_goal(1)-d/2) / (b_length) );

% offset due to thickness of gripper
left_angle_offset = asin( (W/2) /a_length);
right_angle_offset = asin( (W/2) /b_length);

% angle of motor
left_motor_goal = left_angle - left_angle_offset;
right_motor_goal = right_angle - right_angle_offset;

motor_angle_goal = [left_motor_goal, right_motor_goal];

%line(x,y,z, 'Color','black','linewidth',2);


%% Plot goal point square
square_coord_goal = [a_goal; 
    a_goal(1) + L*cos((pi/4)-goal_pose(3)), a_goal(2) + L*sin((pi/4)-goal_pose(3));
    b_goal;
    b_goal(1) - L*cos((pi/4)-goal_pose(3)), b_goal(2) - L*sin((pi/4)-goal_pose(3));
    a_goal;
    b_goal;
    ];

plot(square_coord_goal(:,1), square_coord_goal(:,2), 'b-', 'LineWidth', 0.5);
hold on;

Left_touch_line = [base_left;
    base_left(1) - a_length*cos(left_angle), base_left(2) + a_length*sin(left_angle)];
plot(Left_touch_line(:,1), Left_touch_line(:,2),'r', 'LineWidth', 0.5)
hold on;

Right_touch_line = [base_right;
    base_right(1) + b_length*cos(right_angle), base_right(2) + b_length*sin(right_angle)];
plot(Right_touch_line(:,1), Right_touch_line(:,2),'r', 'LineWidth', 0.5)
hold on;



%% calculate the start pos of rotation that leads to the goal pos
% start from right side

theta1 = asin((W/2)/b_length);
theta2 = theta1 + pi/4;
l1 = sqrt( (square_diagonal)^2 + (b_length)^2 - (cos(theta2)*2*square_diagonal*b_length) );
theta3 = acos( ((d)^2 + (a_length)^2 - (l1)^2) / (2*d*a_length));
alpha_left = pi - theta3;
left_motor_angle1 = alpha_left - left_angle_offset;

theta4 = acos( ((l1)^2+(square_diagonal)^2-(b_length)^2) / (2*l1*square_diagonal) );
theta5 = acos( ((a_length)^2 + (l1)^2 - (d)^2) / (2*a_length*l1));
right_motor_angle1 = pi - (2*pi - theta3 - theta4 - theta5 - pi/4);
alpha_right = right_motor_angle1 + right_angle_offset;

motor_angle1 = [left_motor_angle1, right_motor_angle1];
alpha1 = [alpha_left, alpha_right];

%a1 = [base_left(1) + a_length*cos(theta3), base_left(2) + a_length*sin(theta3)];
a1 = [base_left(1) - a_length*cos(alpha_left), base_left(2) + a_length*sin(alpha_left)];
b1 = [base_right(1) + b_length*cos(alpha_right), base_right(2) + b_length*sin(alpha_right)];
b_low1 = [b1(1)-L*cos(right_motor_angle1), b1(2)-L*sin(right_motor_angle1)];

b_low_length_rotate = sqrt((b_low1(1)-base_right(1))^2+b_low1(2)^2);
a1_length_rotate = sqrt((a1(1)-base_left(1))^2+a1(2)^2);

a1_baseR_radius = sqrt((base_right(1)-a1(1))^2 + (a1(2))^2);
a1_baseR_degree = atan((a1(2))/(base_right(1)-a1(1)));


%% Plot

square_coord1 = [a1;
    b1(1)-L*cos(right_motor_angle1), b1(2)-L*sin(right_motor_angle1);
    b1;
    b1(1)-L*sin(right_motor_angle1), b1(2)+L*cos(right_motor_angle1);
    a1;
    b1;
    ];

plot(square_coord1(:,1), square_coord1(:,2), 'b-', 'LineWidth', 0.5);
xlim([-10 10]);      % set x-axis limits
ylim([-5 20]);      % set y-axis limits
axis equal;
hold on;


Left_touch_line = [base_left; a1];
plot(Left_touch_line(:,1), Left_touch_line(:,2),'r', 'LineWidth', 0.5);
hold on;

Right_touch_line = [base_right; b1];
plot(Right_touch_line(:,1), Right_touch_line(:,2),'r', 'LineWidth', 0.5)
hold on;


d_L = a_length - L;
b = L/2;
a = L/2;
f_w = W/2;
theta_L = left_motor_angle1;

x = (d_L+b)*cos(theta_L) + (a+f_w)*sin(theta_L);
y = (d_L+b)*sin(theta_L) - (a+f_w)*cos(theta_L);



%% rotate to right to obtain a point with already obtained b point

% b_lower_length = sqrt( (L)^2 + (b_length)^2 - (cos(theta1)*2*L*b_length) );
theta6 = acos( ((l1)^2 + (b_length)^2 - (square_diagonal)^2) / (2*l1*b_length) );
theta7 = pi - theta6 - alpha_right;
a2 = [base_right(1)-l1*cos(theta7), l1*sin(theta7)];
% theta7 is the angle needed to obtain a2 with fixed b



%% rotate to left to obtain b point
% a_start is fixed
% a_length is fixed
% alpha3 = pi*3/5;
% a3 = [-d/2-W/2-a_length_start*cos(alpha3) a_length_start*sin(alpha3)];
% left_motor_angle3 = alpha3 - left_offset;
% b_low3 = [a3(1)+L*cos(pi/2-left_motor_angle3) a3(2)+L*sin(pi/2-left_motor_angle3)];
% b3 = [b_low3(1)-L*cos(left_motor_angle3) b_low3(2)+L*sin(left_motor_angle3)];

% for 
% b_low_length = b_low_length_rotate
% a_length = a_length_start
% need to modify b_low_length

l3 = sqrt( (L)^2 + (a_length_start)^2 - (cos(pi/2+left_offset)*2*L*a_length_start) );
theta8 = acos( ((a_length_start)^2 + (l3)^2 - (L)^2) / (2*l3*a_length_start));
theta9 = acos( ((d)^2 + (l3)^2 - (b_low_length_rotate)^2) / (2*l3*d));
alpha3 = pi - theta8 - theta9;
a3 = [base_left(1)-a_length_start*cos(alpha3) a_length_start*sin(alpha3)];
left_motor_angle3 = alpha3 - left_offset;
b_low3 = [a3(1)+L*cos(pi/2-left_motor_angle3) a3(2)+L*sin(pi/2-left_motor_angle3)];
b3 = [b_low3(1)-L*cos(left_motor_angle3) b_low3(2)+L*sin(left_motor_angle3)];

% a3_baseR_degree = atan( a3(2) / (base_right(1)-a3(1)) );

%% plot
% square_coord_3 = [a3; 
%     b_low3;
%     b3
%     a3(1)-L*cos(left_motor_angle3) a3(2)+L*sin(left_motor_angle3);
%     a3;
%     b3
%     ];
% 
% plot(square_coord_3(:,1), square_coord_3(:,2), 'b-', 'LineWidth', 0.5);
% hold on;
% 
% Left_touch_line = [base_left; a3];
% plot(Left_touch_line(:,1), Left_touch_line(:,2),'r', 'LineWidth', 0.5);
% hold on;
% 
% Right_touch_line = [base_right; b3];
% plot(Right_touch_line(:,1), Right_touch_line(:,2),'r', 'LineWidth', 0.5)
% hold on;

%left_angle_offset


%% plot trajectory
radius = a_length_start;            % Radius of the circle
origin = base_left;       % Coordinates of the origin

% [centre3, distance_to_base3, angle_to_base3] = getCentreInfor(square_coord_3(1:4,:), base_right, base_left);
% plotArch (distance_to_base3(1), base_left, pi-left_angle_start, -angle_to_base3(1));

[centre1, distance_to_base1, angle_to_base1] = getCentreInfor(square_coord1(1:4,:), base_right, base_left);
% plotArch (distance_to_base1(2), base_right, pi-angle_to_base1(2),pi-angle_to_base3(2));

% [centre_goal, distance_to_base_goal, angle_to_base_goal] = getCentreInfor(square_coord_goal(1:4,:), base_right, base_left);
% plotArch (distance_to_base_goal(2), base_right, -angle_to_base_goal(2),-angle_to_base1(2));

%% plotting setting
xlim([-10 10]);      % set x-axis limits
ylim([-5 10]);      % set y-axis limits
axis equal;
grid on;



function [] = plotArch (radius, origin, start_angle, end_angle)
    
    % Generate a set of angles to plot the circle
    theta = linspace(start_angle, end_angle, 100);
    
    % Calculate the x and y coordinates of the circle
    x = origin(1) + radius * cos(theta);
    y = origin(2) + radius * sin(theta);
    
    % Plot the circle
    plot(x, y, 'k', 'LineWidth', 2);
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

function [] = plotSqaureAndTouch ()
end