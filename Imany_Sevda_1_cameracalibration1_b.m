clc; clear; close all;

load('./outputs/homographies_myimages.mat', 'refined_homographies');
homography_subset = refined_homographies(1:6);  % Use 6 homographies

% Compute intrinsic matrix
A = internal_parameters_solve_vmmc(homography_subset);

% Extract components for display
results_refined = [A(1,1), A(2,2), A(1,3), A(2,3), A(1,2)];

% Display results
fprintf('\nIntrinsic Camera Parameters (After Refinement):\n');
fprintf('| # of Homographies | Scale Factor (a) | Scale Factor (b) | Principal Point u0 | Principal Point v0 | Skew |\n');
fprintf('|-------------------|------------------|------------------|--------------------|--------------------|------|\n');
fprintf('| %d                 | %.2f          | %.2f          | %.2f             | %.2f             |%.2f|\n', ...
    6, results_refined(1), results_refined(2), results_refined(3), results_refined(4), results_refined(5));

% save('./outputs/mobile_intrinsic_parameters.mat', 'results_refined');
save('./outputs/intrinsic_matrix_A.mat', 'A');
dlmwrite('./outputs/intrinsic_matrix_A.txt', A, 'delimiter', '\t', 'precision', '%.6f');
