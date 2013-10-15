% function F = compute_F_matrix(points1, points2);
%
% Method:   Calculate the F matrix between two views from
%           point correspondences: points2^T * F * points1 = 0
%           We use the normalize 8-point algorithm and 
%           enforce the constraint that the three singular 
%           values are: a,b,0. The data will be normalized here. 
%           Finally we will check how good the epipolar constraints:
%           points2^T * F * points1 = 0 are fullfilled.
% 
%           Requires that the number of cameras is C=2.
% 
% Input:    points2d is a 3xNxC array storing the image points.
%
% Output:   F is a 3x3 matrix where the last singular value is zero.

function F = compute_F_matrix( points2d )

N = compute_normalization_matrices(points2d);

points2d_norm_1 = N(:,:,1) * points2d(:,:,1);
points2d_norm_2 = N(:,:,2) * points2d(:,:,2);

xa = points2d_norm_1(1,:);
xb = points2d_norm_2(1,:);
ya = points2d_norm_1(2,:);
yb = points2d_norm_2(2,:);


Q = [xb.*xa;
    xb.*ya;
    xb;
    yb .* xa;
    yb .* ya;
    yb;
    xa;
    ya;
    ones(1,size(points2d, 2))].';   

[~, ~, V] = svd(Q);
    
g = V(:,end);
G = [g(1:3).'; g(4:6).'; g(7:9).'];

F = N(:,:,2).' * G * N(:,:,1);

[U, S, V] = svd(F);

% S_new = [S(1,1), 0,         0;
%      0,         S(2,2), 0;
%      0,         0,         0];
S(3,3) = 0;

F = U * S * V.';

