%% Matching points using Matlab functions
cimg1 = imread('a1.jpg');
cimg2 = imread('a2.jpg');
% cimg1 = imread('canny_a1.jpg');
% cimg2 = imread('canny_a2.jpg');

img1 = rgb2gray(cimg1);
img2 = rgb2gray(cimg2);

Feature_points1 = detectSURFFeatures(img1);
Feature_points2 = detectSURFFeatures(img2);

[Features1, Feature_points1] = extractFeatures(img1, Feature_points1);
[Features2, Feature_points2] = extractFeatures(img2, Feature_points2);

Pairs = matchFeatures(Features1, Features2);

Matched_points1 = Feature_points1(Pairs(:, 1), :);
Matched_points2 = Feature_points2(Pairs(:, 2), :);
Num_mp = Matched_points1.size(1);
disp(sprintf('%d points are matched!', Num_mp))

%% Get Fundamental Matrix

%homogeneous coordinates
mph1 = double([Matched_points1.Location, ones(Matched_points1.size)]);
mph2 = double([Matched_points2.Location, ones(Matched_points2.size)]);

%figure pos
Fig = figure(1);
set(Fig, 'Position', [500 100 960 1150])


% show Matched points
subplot(1,2,[1 2]);
showMatchedFeatures(cimg1, cimg2, Matched_points1, Matched_points2, 'montage');
title('Putatively Matched Points (Including Outliers)');

% % 8 point algorithm
% F_8point = norm8point(mph1, mph2, -1);
% 
% % normalize 8 point algorithm
% F_norm8point = norm8point(mph1, mph2, 1);
% 
% % non-linear optimization
% F_nonlinear = non_linear(mph1, mph2, rand(3,3));
% 
% 
% % RANSAC
% Max_iter = 1000;
% Num_rp = 10;
% Best_Num_inlier = 0;
% D_Th = 0.05;
% F_RANSAC = zeros(3,3);
% 
% for i = 1:Max_iter
%     shuffle = randperm(Num_mp);
%     rmph1 = mph1(shuffle(1:Num_rp),:);
%     rmph2 = mph2(shuffle(1:Num_rp),:);
%     F_can = norm8point(rmph1, rmph2, 1);
%     %F_can = non_linear(rmph1, rmph2, rand(3,3));
%     D = (mph1*F_can*mph2');
%     D = sqrt(D(1:size(D,1)+1:end)'.^2);
%     inlier_idx = D < D_Th;
%     Num_inlier = sum(inlier_idx);
% 
%     if (Num_inlier > Best_Num_inlier)
%         Best_Num_inlier = Num_inlier;
%         F_RANSAC = F_can;
%         bp1 = rmph1;
%         bp2 = rmph2;
%         best_inliers = inlier_idx;
%     end
% end
% % Show Matched points (inliers)
% subplot(3,2,[3 4]);
% showMatchedFeatures(cimg1, cimg2, Matched_points1(best_inliers,:), Matched_points2(best_inliers,:), 'montage');
% title('Matched Points (only inliers)');
% 
% 
% 
% % Plot Epipolar Line
% % 
% % F = F_RANSAC;
% % 
% % while(1)
% % subplot(3,2,5);
% % imshow(cimg1)
% % title('Image1 : Click point');
% % [x,y] = ginput(1);
% % eline = [x, y, 1] * F';
% % 
% % subplot(3,2,6);
% % points = lineToBorderPoints(eline, size(cimg2));
% % imshow(cimg2);
% % line(points(:, [1,3])', points(:, [2,4])');
% % title('Image2 with epipolar line')
% % end