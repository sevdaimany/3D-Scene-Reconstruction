clc; clear; close all;
warning off

ACT_path = './toolboxes/ACT_lite/';
addpath(genpath(ACT_path));
extra_funs_path = './toolboxes/extra_funs/';
addpath(genpath(extra_funs_path));

load('./outputs/point_matrix_session3.mat');
[~, npoints, numViews] = size(point_matrix);
q_data = homogenize_coords(point_matrix);
ncam = numViews;

load('./outputs/intrinsic_matrix_A.mat');
K = repmat(A, 1, 1, ncam);  % 3x3xncam

imageFolder = './images/session3/'; 
for i = 1:numViews
    images{i} = imread(fullfile(imageFolder, sprintf('image_%d.jpg', i)));
end




% 2. Compute the fundamental matrix using the first and last cameras
% of the camera set (N cameras)
% ------------------------------------------------------------------------
q_2cams(:,:,1)=q_data(:,:,1); 
q_2cams(:,:,2)=q_data(:,:,ncam);
 
[F, P_2cam_est,Q_2cam_est,q_2cam_est] = MatFunProjectiveCalib(q_2cams);

pts1 = q_2cams(1:2,:,1)';
pts2 = q_2cams(1:2,:,2)';

img1 = images{1};
img2 = images{ncam};

I12 = cat(2, img1, img2);
figure; imshow(uint8(I12)); hold on;
pos1 = q_2cams(1:2,:,1)';
pos2 = q_2cams(1:2,:,2)';
plot([pos1(:,1), pos2(:,1)+size(img1,2)]', [pos1(:,2), pos2(:,2)]','-');
plot([pos1(:,1), pos2(:,1)+size(img1,2)]', [pos1(:,2), pos2(:,2)]','o');

disp(['Residual reprojection error. 8 point algorithm   = ' num2str( ErrorRetroproy(q_2cams,P_2cam_est,Q_2cam_est)/2 )]);
draw_reproj_error(q_2cams,P_2cam_est,Q_2cam_est);
title('Residual reprojection initial');

% ------------------------------------------------------------------------
% 3. Resection. Obtain the projection matrices of the rest of cameras using the PDLT_NA function 
% ------------------------------------------------------------------------

P_all = zeros(3, 4, ncam);
P_all(:,:, 1) = P_2cam_est(:,:,1);
P_all(:,:, ncam) = P_2cam_est(:,:,2);

for i = 2:ncam-1 
    x = q_data(:,:,i); 
    X = Q_2cam_est; 
    P_all(:,:,i)  = PDLT_NA(x, X, 0, 0);
end

disp(['Residual reprojection after Resection = ' num2str( ErrorRetroproy(q_data,P_all,Q_2cam_est)/2 )]);
draw_reproj_error(q_data, P_all,Q_2cam_est);
title('Residual reprojection after Resection');
% ------------------------------------------------------------------------
% 3. Projective Bundle Adjustment. Use BAProjectiveCalib function
% Coordinates of 3D and 2D points are given in homogeneus coordinates
% ------------------------------------------------------------------------
% auxiliary matrix that indicates that all points are visible in all the cameras
vp = ones(npoints,ncam);
[P_refined,X3d,xc] = BAProjectiveCalib(q_data,P_all,Q_2cam_est,vp);

disp(['Residual reprojection after Bundle Adjustment  = ' num2str( ErrorRetroproy(q_data,P_refined,X3d)/2 )]);
draw_reproj_error(q_data,P_refined,X3d);
title('Residual reprojection after Bundle Adjustment');

% ------------------------------------------------------------------------
% 4. Re-compute the Fundamental matrix between two of the cameras, using the projection matrices
% obtained after the Projective Bundle Adjustment step
% ------------------------------------------------------------------------
F_refined = vgg_F_from_P(P_refined(:,:,1), P_refined(:,:,ncam));
display(F_refined);

% ------------------------------------------------------------------------
% 5. Use the properties of the Essential matrix (between two cameras) to obtain a Euclidean reconstruction
% of the scene.
% ------------------------------------------------------------------------

% Compute the Essential matrix from the Fundamental matrix
K_2cameras = zeros(3, 3, 2);
K_2cameras(:,:,1) = K(:,:, 1);
K_2cameras(:,:,2) = K(:,:, ncam);
E = K_2cameras(:,:, 2)' * F_refined * K_2cameras(:, : ,1);
disp('Essential Matrix:');
disp(E);

[R_est,T_est] = factorize_E(E);
R1 = R_est(:,:,1);
t1 = T_est;

% Build full R and t arrays for draw_scene
R_euc(:,:,1) = eye(3);   % First camera
R_euc(:,:,2) = R1;       % Second camera

t_euc(:,1) = zeros(3,1); % First camera
t_euc(:,2) = t1;         % Second camera


% Triangulate 3D points using known solution

x_2cameras(:,:, 1) = xc(:,:, 1);
x_2cameras(:,:, 2) = xc(:,:, ncam);

Q_euc = TriangEuc(R1, t1, K_2cameras, x_2cameras);

% visualize 3D reconstruction
figure();
draw_scene(Q_euc, K_2cameras, R_euc, t_euc);
title('Euclidean 3D Scene of 2 cameras using essential matrix');

% Compute the projection matrices from K, Rcam, Tcam
for k=1:2
    Rt = [R_euc(:,:,k), -R_euc(:,:,k) * t_euc(:,k)];
    P_euc(:,:,k) = K_2cameras(:,:,k) * Rt;
end

for k=1:2
    q_rep(:,:, k) = P_euc(:, : ,k) * Q_euc;
end


% ------------------------------------------------------------------------
% 6. Resection. Obtain the projection matrices of the rest of cameras using the PDLT_NA function 
% ------------------------------------------------------------------------

P_all = zeros(3, 4, ncam);
P_all(:,:, 1) = P_euc(:,:,1);
P_all(:,:, ncam) = P_euc(:,:,2);
for i = 2:ncam-1 
    x = xc(:,:,i); 
    X = Q_euc; 
    P_all(:,:,i)  = PDLT_NA(x, X, 0, 0);
end

disp(['Residual reprojection after Resection = ' num2str( ErrorRetroproy(xc,P_all,Q_euc)/2 )]);
draw_reproj_error(xc, P_all,Q_euc);
title('Residual reprojection after Resection');


% ------------------------------------------------------------------------
% 3. Projective Bundle Adjustment. Use BAProjectiveCalib function
% Coordinates of 3D and 2D points are given in homogeneus coordinates
% ------------------------------------------------------------------------
% auxiliary matrix that indicates that all points are visible in all the cameras
vp = ones(npoints,ncam);
[P_refined,X3d,xc] = BAProjectiveCalib(xc,P_all,Q_euc,vp);

disp(['Residual reprojection after Bundle Adjustment  = ' num2str( ErrorRetroproy(xc,P_refined,X3d)/2 )]);
draw_reproj_error(xc,P_refined,X3d);
title('Residual reprojection after Bundle Adjustment');


% --- Decompose cameras into intrinsics and extrinsics ---
[K_euc, R_euc, C_euc] = CameraMatrix2KRC(P_refined);

% --- scene visualize 
figure;
draw_scene(X3d, K_euc, R_euc, C_euc(1:3,:));
title('Refined 3D Scene after Final Bundle Adjustment');

% Draw 3D points
Q = un_homogenize_coords(X3d);
scatter3(Q(1,:),Q(2,:),Q(3,:));
daspect([1, 1, 1]); pbaspect([1, 1, 1]); axis vis3d;
title('Final 3D Scene');



