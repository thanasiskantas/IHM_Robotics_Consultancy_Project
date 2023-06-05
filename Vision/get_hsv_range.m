clear all
clc

img = imread ("/Users/qiyangyan/Desktop/Vision Robotic Assembly Lines/ComputerVisionPart/test gripper_block imgs/gripper5.jpeg");
img = imgaussfilt(img);
img = imrotate(img, -90);


% get mask
hsv_img = rgb2hsv(img);
mask = getMask(hsv_img);
mask = medfilt2(mask);
seOpening = strel('disk', 5);
mask = imopen(mask,seOpening);
mask = imclose(mask,seOpening);


% Calculate the centroid using regionprops, based on the bounding box
s = regionprops(mask, 'Centroid');
centroid = s.Centroid;
centroidX = centroid(1);
centroidY = centroid(2);
disp(['Centroid coordinates: (', num2str(centroidX), ', ', num2str(centroidY), ')']);

% % Calculate the bounding box using regionprops
% s = regionprops(mask, 'BoundingBox');
% boundingBox = s.BoundingBox;
% 
% % Display the bounding box on the original image
% rectangle('Position', boundingBox, 'EdgeColor', 'r', 'LineWidth', 2);
% hold off;

figure;
% subplot(2,1,1);
imshow(img)
% subplot(2,1,2);
% imshow(mask)
hold on;


% draw contour
contourData = bwboundaries(mask);
for k = 1:numel(contourData)
    contour = contourData{k};
    plot(contour(:,2), contour(:,1), 'y', 'LineWidth', 2);
end


s = regionprops(mask, 'Orientation');
% Access the orientation value
orientation = s.Orientation;

% Display the orientation value
disp(['Orientation: ', num2str(orientation)]);

% Plot the object with orientation
% imshow(binaryMask);
% hold on;

% Calculate the line endpoints based on the centroid and orientation
lineLength = 150; % Adjust the line length as needed
x1 = centroidX - lineLength * cosd(orientation);
y1 = centroidY - lineLength * sind(orientation);
x2 = centroidX + lineLength * cosd(orientation);
y2 = centroidY + lineLength * sind(orientation);

% Plot the line representing the orientation
plot([x1, x2], [y1, y2], 'k', 'LineWidth', 2);

plot(centroidX, centroidY, 'r+', 'MarkerSize', 10);

hold off;


%% functions

function mask = getMask(hsv_img) 

    %light_green = [40, 60, 25] ./ 255; % only block
    light_green = [40, 60, 60] ./ 255; % within gripper
    dark_green = [99, 255, 255] ./ 255;

    % light_purple = [110, 43, 46] ./ 255;
    % dark_purple = [155, 255, 255] ./ 255;
    % 
    % light_yellow = [18, 65, 46] ./ 255;
    % dark_yellow = [30, 255, 255] ./ 255;
    % 
    % light_red = [0, 43, 46] ./ 255;
    % dark_red = [5, 255, 255] ./ 255;
    % 
    % light_blue = [100, 60, 50] ./ 255;
    % dark_blue = [117, 255, 255] ./ 255;
    
    % 提取H、S、V通道
    H = hsv_img(:, :, 1);
    S = hsv_img(:, :, 2);
    V = hsv_img(:, :, 3);
    
    % get mask
    mask = (H > light_green(1) & H < dark_green(1)) & (S > light_green(2) & S < dark_green(2)) & (V > light_green(3) & V < dark_green(3));

end


function plot_hsv_spectrum(img)
    hsv = rgb2hsv(img);
    h = hsv(:,:,1);
    s = hsv(:,:,2);
    v = hsv(:,:,3);
    figure;
    subplot(1,3,1);
    imshow(h);
    title('Hue');
    subplot(1,3,2);
    imshow(s);
    title('Saturation');
    subplot(1,3,3);
    imshow(v);
    title('Value');
end