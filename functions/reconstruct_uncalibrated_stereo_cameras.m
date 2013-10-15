% function [cams, cam_centers] = reconstruct_uncalibrated_stereo_cameras(F); 
%
% Method: Calculate the first and second uncalibrated camera matrix
%         from the F-matrix. 
% 
% Input:  F - Fundamental matrix with the last singular value 0 
%
% Output:   cams is a 3x4x2 array, where cams(:,:,1) is the first and 
%           cams(:,:,2) is the second camera matrix.
%
%           cam_centers is a 4x2 array, where (:,1) is the first and (:,2) 
%           the second camera center.

function [cams, cam_centers] = reconstruct_uncalibrated_stereo_cameras( F )

% cams = zeros(3,4,2);
% cam_centers = zeros(4,2);

Ma = [eye(3), zeros(3,1)];
cams(:,:,1) = Ma;
cam_centers(:,1) = [zeros(3,1); 1];


W = [0, -1, 1;
     1,  0, -1;
     -1,  1, 0];
 


[~, ~, V] = svd(F');

h = V(:,end);

Mb = [W*F, h];
cams(:,:,2) = Mb;

[~, ~, V] = svd(Mb);

cam_centers(:,2) = V(:,end);