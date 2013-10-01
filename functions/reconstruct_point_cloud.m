% function model = reconstruct_point_cloud(cam, data)
%
% Method:   Determines the 3D model points by triangulation
%           of a stereo camera system. We assume that the data 
%           is already normalized 
% 
%           Requires that the number of cameras is C=2.
%           Let N be the number of points.
%
% Input:    points2d is a 3xNxC array, storing all image points.
%
%           cameras is a 3x4xC array, where cameras(:,:,1) is the first and 
%           cameras(:,:,2) is the second camera matrix.
% 
% Output:   points3d 4xN matrix of all 3d points.


function points3d = reconstruct_point_cloud( cameras, points2d )

%------------------------------
% TODO: FILL IN THIS PART

xa = points2d(1,:);
xb = points2d(1,:);
ya = points2d(2,:);
yb = points2d(2,:);

ma = cameras(:,:,1);
mb = cameras(:,:,2);

q1 = [xa.*ma(3,1)-ma(1,1); xa.*ma(3,2)-ma(1,2); xa.*ma(3,3)-ma(1,3); xa.*ma(3,4)-ma(1,4)].';
q2 = [ya.*ma(3,1)-ma(2,1): ya.*ma(3,2)-ma(2,2); ya.*ma(3,3)-ma(2,3); ya.*ma(3,4)-ma(2,4)].';
q3 = [xb.*ma(3,1)-ma(1,1); xb.*ma(3,2)-ma(1,2); xb.*ma(3,3)-ma(1,3); xb.*ma(3,4)-ma(1,4)].';
q4 = [yb.*ma(3,1)-ma(2,1): yb.*ma(3,2)-ma(2,2); yb.*ma(3,3)-ma(2,3); yb.*ma(3,4)-ma(2,4)].';

Q = [q1; q2; q3; q4];
[U S V] = svd(Q);

v = V(:,end);

points3d = [v(1:4).'; v(5:8).'; v(9:12); v(13:16).'];
