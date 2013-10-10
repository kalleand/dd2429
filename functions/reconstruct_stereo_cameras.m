% function [cams, cam_centers] = reconstruct_stereo_cameras(E, K1, K2, data); 
%
% Method:   Calculate the first and second camera matrix. 
%           The second camera matrix is unique up to scale. 
%           The essential matrix and 
%           the internal camera matrices are known. Furthermore one 
%           point is needed in order solve the ambiguity in the 
%           second camera matrix.
%
%           Requires that the number of cameras is C=2.
%
% Input:    E is a 3x3 essential matrix with the singular values (a,a,0).
%
%           K is a 3x3xC array storing the internal calibration matrix for
%           each camera.
%
%           points2d is a 3x1xC matrix, storing an image point for each camera.
%
% Output:   cams is a 3x4x2 array, where cams(:,:,1) is the first and 
%           cams(:,:,2) is the second camera matrix.
%
%           cam_centers is a 4x2 array, where (:,1) is the first and (:,2) 
%           the second camera center.
%

function [cams, cam_centers] = reconstruct_stereo_cameras( E, K, points2d )

%------------------------------
% TODO: FILL IN THIS PART

Ma = [eye(3), zeros(3,1)];
Ka = K(:,:,1);
Kb = K(:,:,2);
cams(:,:,1) = Ka * Ma;
cam_centers(:,1) = [zeros(3,1); 1];
 
W = [0, -1, 0;
     1,  0, 0;
     0,  0, 1];
 
[U, ~, V] = svd(E);

R1 = U * W * V';
R2 = U * W' * V';

R1 = R1 * det(R1);
R2 = R2 * det(R2);

t = V(:,end);


RIt(:,:,1) = R1 * [eye(3), t];
RIt(:,:,2) = R1 * [eye(3), -t];
RIt(:,:,3) = R2 * [eye(3), t];
RIt(:,:,4) = R2 * [eye(3), -t];

p = zeros(4,1,4);
cams_tmp(:,:,1) = Ka * Ma;

for i = 1:4

    cams_tmp(:,:,2) = Kb * RIt(:,:,i);
    p(:,1,i) = reconstruct_point_cloud(cams_tmp, points2d(:,1,:));
    p(:,1,i) = [homogeneous_to_cartesian(p(:,1,i)); 1];
end
for i = 1:4
    gp = RIt(:,:,i) * p(:,1,i);
    

    if sign(p(3,:,i)) == 1
        if sign(gp(3)) == 1
            cams(:,:,2) = Kb * RIt(:,:,i);
            fprintf('Using alternative %d\n', i);
            if i == 1 || i == 3
                cam_centers(:,2) = [-t; 1];
            else
                cam_centers(:,2) = [t; 1];
            end
        end
    end
end
