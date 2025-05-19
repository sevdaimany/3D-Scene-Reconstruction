clc; clear; close all;

% n-view matching for 3D reconstruction

numImages = 4;

images = cell(1, numImages);
keypoints_all = cell(1, numImages);
descriptors_all = cell(1, numImages);

imageFolder = './images/session3/'; 
for i = 1:numImages
    images{i} = imread(fullfile(imageFolder, sprintf('image_%d.jpg', i)));
end

detection_mode = 1;

for i = 1:numImages
    imgGray = single(rgb2gray(images{i}));
    
    kp = detectSIFTFeatures(uint8(imgGray));
    [features, valid_kp] = extractFeatures(uint8(imgGray), kp);
    keypoints_all{i} = valid_kp;
    descriptors_all{i} = features;
    
    
end

% === Match Features Across Views Consistently ===
MaxRatio = 0.7;         
Metric = 'SSD';        

point_matrix = n_view_matching(keypoints_all, descriptors_all, images, MaxRatio, Metric);

[~, numPoints, numViews] = size(point_matrix);

fprintf('Total points: %d\n', numPoints);
fprintf('Total views: %d\n\n', numViews);

save('point_matrix_session3.mat', 'point_matrix');
