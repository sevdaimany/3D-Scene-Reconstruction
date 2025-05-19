clc; clear; close all;
warning off

detection_mode = 0; % 0:Matlab sift, 1:VLFeat sift
matching_mode = 0; % 0:MATLAB matchFeatures, 1:VLFeat, 2:nndr

% VLFeat SIFT
run('toolboxes/vlfeat-0.9.21/toolbox/vl_setup') 



% load images with montage
imageFolder = './images/session3/'; 
imageFiles = dir(fullfile(imageFolder, '*.jpg'));
images = cell(1, length(imageFiles));
for i = 1:length(imageFiles)
    images{i} = imread(fullfile(imageFolder, imageFiles(i).name));
end
% Show all views
figure; montage(images); title('Captured Scene Views');


imgPath1 = './images/session3/image_1.jpg';
imgPath2= './images/session3/image_2.jpg';
% imgPath1 = './images/session3/image_2.jpg';
% imgPath2= './images/session3/shimage_3.jpg';

% Setups to evaluate
setups = {
    struct('det', 0, 'match', 0, 'name', 'MATLAB SIFT + MATLAB matchFeatures', ...
           'params', struct('MatchThreshold', 100, 'MaxRatio', 0.1)),
          
    struct('det', 0, 'match', 0, 'name', 'MATLAB SIFT + MATLAB matchFeatures', ...
           'params', struct('MatchThreshold', 50, 'MaxRatio', 0.05)),
    
    struct('det', 1, 'match', 1, 'name', 'VLFeat SIFT + VLFeat matcher', ...
           'params', struct('TopNMatches', 150)),
    struct('det', 1, 'match', 1, 'name', 'VLFeat SIFT + VLFeat matcher', 'params', struct('TopNMatches', 300)),

    struct('det', 0, 'match', 2, 'name', 'MATLAB SIFT + Custom NNDR', ...
           'params', struct('NNDRThreshold', 0.3)),

    struct('det', 1, 'match', 2, 'name', 'VLFeat SIFT + Custom NNDR', ...
           'params', struct('NNDRThreshold', 0.3)),
    
    struct('det', 0, 'match', 2, 'name', 'MATLAB SIFT + Custom NNDR', ...
           'params', struct('NNDRThreshold', 0.2)),
    struct('det', 1, 'match', 2, 'name', 'VLFeat SIFT + Custom NNDR', ...
           'params', struct('NNDRThreshold', 0.2))
};

% Store results
resultsTable = [];

for i = 1:length(setups)
    s = setups{i};
    fprintf('Running setup: %s\n', s.name);
    res = evaluate_feature_matching(imgPath1, imgPath2, s.det, s.match, s.name, ...
        s.params);

    paramStr = jsonencode(s.params); 
    inlierRatioH = res.numInliersH / res.numMatches;
    inlierRatioF = res.numInliersF / res.numMatches;

    resultsTable = [resultsTable; {
        s.name, ...
        res.numMatches, ...
        res.numInliersF, ...
        res.numInliersH, ...
        inlierRatioF, ...
        inlierRatioH, ...
        ~res.homographyStatus, ...  % 1 if successful
        paramStr
    }];
end

% Display comparison table
T = cell2table(resultsTable, ...
    'VariableNames', {'Setup', 'NumMatches', 'NumInliers_FMatrix', 'NumInliers_Homography', 'InlierRatio_FMatrix','InlierRatio_Homography', 'HomographySuccess', 'Parameters'});
disp(T);
% writetable(T, 'results_table_image2_3.txt', 'Delimiter', '\t');



function results = evaluate_feature_matching(imgPath1, imgPath2, detection_mode, matching_mode, name, varargin)
% evaluate_feature_matching - Compare SIFT detection and matching setups.
%
% Inputs:
%   imgPath1        - path to first image
%   imgPath2        - path to second image
%   detection_mode  - 0: MATLAB SIFT, 1: VLFeat SIFT
%   matching_mode   - 0: MATLAB matchFeatures, 1: VLFeat, 2: custom NNDR
%   varargin        - additional params: MatchThreshold, MaxRatio, NNDRThreshold
%
% Output:
%   results - struct containing matches, matrices, and images

    % --- Default Parameters ---
    p = inputParser;    
    addParameter(p, 'MatchThreshold', 100);
    addParameter(p, 'MaxRatio', 0.2);
    addParameter(p, 'NNDRThreshold', 0.6);
    addParameter(p, 'TopNMatches', 1000);
    parse(p, varargin{:});
    opts = p.Results;


    % --- Setup & Load Images ---
    run('toolboxes/vlfeat-0.9.21/toolbox/vl_setup');
    img_color1 = imread(imgPath1);
    img_color2 = imread(imgPath2);
    img1 = single(rgb2gray(img_color1));
    img2 = single(rgb2gray(img_color2));
    img_gray2 = rgb2gray(img_color2);

    % --- Keypoint Detection ---
    if detection_mode == 0
        kp1 = detectSIFTFeatures(rgb2gray(img_color1));
        kp2 = detectSIFTFeatures(rgb2gray(img_color2));
        [features1, kp1] = extractFeatures(rgb2gray(img_color1), kp1);
        [features2, kp2] = extractFeatures(rgb2gray(img_color2), kp2);
        f1 = matlab2vlfeat(kp1);
        f2 = matlab2vlfeat(kp2);
        d1 = features1';
        d2 = features2';
    else
        [f1, d1] = vl_sift(img1);
        [f2, d2] = vl_sift(img2);
    end

    % --- Feature Matching ---
    switch matching_mode
        case 0 % MATLAB matchFeatures
            indexPairs = matchFeatures(d1', d2', 'MatchThreshold', opts.MatchThreshold, 'MaxRatio', opts.MaxRatio);
            idx1 = indexPairs(:,1);
            idx2 = indexPairs(:,2);
        case 1 % VLFeat
            [matches, scores] = vl_ubcmatch(d1, d2);
            
            % Sort by score and select top-N matches
            [~, sortedIdx] = sort(scores, 'ascend');
            matches = matches(:, sortedIdx(1:min(opts.TopNMatches, size(matches,2))));

            idx1 = matches(1,:)';
            idx2 = matches(2,:)';
        case 2 % Custom NNDR
            [matches, ~, ~] = my_nndr_match(double(d1'), double(d2'), opts.NNDRThreshold);
            idx1 = matches(:,1);
            idx2 = matches(:,2);
        otherwise
            error('Unknown matching_mode');
    end

    % --- Visualize Matches ---
    I12 = cat(2, img1, img2);
    figure; imshow(uint8(I12)); hold on;
    pos1 = f1(1:2, idx1)';
    pos2 = f2(1:2, idx2)';
    plot([pos1(:,1), pos2(:,1)+size(img1,2)]', [pos1(:,2), pos2(:,2)]','-');
    plot([pos1(:,1), pos2(:,1)+size(img1,2)]', [pos1(:,2), pos2(:,2)]','o');
    title(sprintf('%s Feature Matches: %d pairs',  name, length(idx1)));

    % --- Estimate Homography ---
    matchedPoints1 = f1(1:2, idx1)';
    matchedPoints2 = f2(1:2, idx2)';
    [tformH, inliersH, statusH] = estimateGeometricTransform2D(matchedPoints1, matchedPoints2, 'projective', ...
        'MaxNumTrials', 2000, 'Confidence', 99.9);
    img1_warped = imwarp(img_color1, tformH, 'OutputView', imref2d(size(img_gray2)));
    figure; imshowpair(img1_warped, img_color2, 'blend');
    title(sprintf('%s — Warped Image (Homography)', name));
    fprintf('%s Estimated Homography:\n', name);
    disp(tformH.T);

    % --- Estimate Fundamental Matrix ---
    [fMatrix, inliersF] = estimateFundamentalMatrix(matchedPoints1, matchedPoints2, ...
        'Method', 'RANSAC', 'NumTrials', 2000, 'DistanceThreshold', 1);
    fprintf('%s Estimated Fundamental Matrix:\n', name);
    disp(fMatrix);
    vgg_gui_F(img_color1, img_color2, fMatrix);

    % --- Output Results ---
    results = struct();
    results.f1 = f1;
    results.f2 = f2;
    results.idx1 = idx1;
    results.idx2 = idx2;
    results.fMatrix = fMatrix;
    results.homography = tformH;
    results.homographyStatus = statusH;
    results.numMatches = length(idx1);
    results.numInliersF = sum(inliersF);
    results.numInliersH = sum(inliersH);

end



function vl_kp = matlab2vlfeat(matlab_kp)
    % Input: MATLAB SIFTPoints object
    % Output: VLFeat format [x; y; scale; orientation] 
    locations = matlab_kp.Location';
    scales = matlab_kp.Scale';
    orientations = matlab_kp.Orientation';
    
    vl_kp = [locations(1,:); locations(2,:); scales; orientations];
end



function [matches, distances, nndrs] = my_nndr_match(descriptors1, descriptors2, nndr_th)
    matches = [];
    distances = [];
    nndrs = [];
    
    for i = 1:size(descriptors1,1)
        d1 = descriptors1(i,:);
        dists = sqrt(sum((descriptors2 - d1).^2, 2));
        [sorted, idx] = sort(dists);
        
        nndr = sorted(1) / sorted(2);
        if nndr < nndr_th
            matches(end+1,:) = [i, idx(1)];
            distances(end+1) = sorted(1);
            nndrs(end+1) = nndr;
        end
    end
end