clc; clear; close all;

num_images = 6;
num_points = 9;

% checkerboard_width = 193.5;  
% [real_points, pattern_image] = get_real_points_checkerboard_vmmc(num_points, checkerboard_width, false);

% Session 1-2  =============================================================
checkerboard_width = 240;  
real_points = [
    240, 0;
    240, 240;
    173, 65;
    173, 180;
    125, 112;
    67, 62;
    67, 188;
    0, 0;
    0, 240
]; 
% === Create pattern_image to visualize ===
pattern_size = 300;
pattern_image = ones(pattern_size, pattern_size); 
scale = 1;  
offset = 20;  
x = real_points(:,1) * scale + offset;
y = pattern_size - (real_points(:,2) * scale + offset);  
for i = 1:num_points
    pattern_image = insertShape(pattern_image, 'FilledCircle', [x(i), y(i), 5], 'Color', 'black', 'Opacity', 1);
end

pattern_image = im2double(rgb2gray(pattern_image));

figure;
imshow(pattern_image);
hold on;
for i = 1:num_points
    text(x(i)+5, y(i), sprintf('%d', i), 'Color', 'red', 'FontSize', 12);
end
title('Custom Pattern Visualization');
%========================================================================


homographies = cell(1, num_images);
refined_homographies = cell(1, num_images);
for j = 1:num_images
    image_filename = sprintf('images/session1_b/image_%d.jpg', j);
    img = imread(image_filename);

    projected_points = get_user_points_vmmc(img);
    H = homography_solve_vmmc(real_points', projected_points);
    homographies{j} = H;
    
    [H_refined, reprojection_error] = homography_refine_vmmc(real_points', projected_points, H);
    refined_homographies{j} = H_refined;
    
    T2 = maketform("projective", H');

    transformed_pattern = imtransform(pattern_image, T2);
    figure;
    subplot(121);
    imshow(img);
    title(['Pattern ', num2str(j)]);
    subplot(122);
    imshow(transformed_pattern);
    title(['Transformed Pattern ', num2str(j)]);
    
    fprintf('Reprojection error for Image %d: %.4f\n', j, reprojection_error);
end

save('./outputs/homographies_myimages_b.mat', 'refined_homographies');
