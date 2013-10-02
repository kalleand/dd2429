% function E = compute_E_matrix( points1, points2, K1, K2 );
%
% Method:   Calculate the E matrix between two views from
%           point correspondences: points2^T * E * points1 = 0
%           we use the normalize 8-point algorithm and 
%           enforce the constraint that the three singular 
%           values are: a,a,0. The data will be normalized here. 
%           Finally we will check how good the epipolar constraints:
%           points2^T * E * points1 = 0 are fullfilled.
% 
%           Requires that the number of cameras is C=2.
% 
% Input:    points2d is a 3xNxC array storing the image points.
%
%           K is a 3x3xC array storing the internal calibration matrix for
%           each camera.
%
% Output:   E is a 3x3 matrix with the singular values (a,a,0).

function E = compute_E_matrix( points2d, K )

%------------------------------
% TODO: FILL IN THIS PART

points2d_1 = K(:,:,1) \ points2d(:,:,1);
points2d_2 = K(:,:,2) \ points2d(:,:,2);

p(:,:,1) = points2d_1;
p(:,:,2) = points2d_2;

N = compute_normalization_matrices(p);

points2d_norm_1 = N(:,:,1) * points2d_1;
points2d_norm_2 = N(:,:,2) * points2d_2;

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
    
f = V(:,end);
F = [f(1:3).'; f(4:6).'; f(7:9).'];

E = N(:,:,2).' * F * N(:,:,1);

[U, S, V] = svd(E);

S_correct = (S(1,1) + S(2,2))/ 2;

S_new = [S_correct, 0,         0;
     0,         S_correct, 0;
     0,         0,         0];

E = U * S_new * V.';


























