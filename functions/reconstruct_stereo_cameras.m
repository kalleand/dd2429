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

cams(:,:,1) = K(:,:,1) * [eye(3), zeros(3,1)];

cam_centers(:,1) = [zeros(3,1); 1];

W = [0, -1, 0;
     1,  0, 0;
     0,  0, 1];
 
[U, ~, V] = svd(E);

R1 = U * W * V';
R2 = U * W' * V';

d1 = det(R1);
d2 = det(R2);

if d1 == -1
    R1 = -1 * R1;
end
if d2 == -1
    R2 = -1 * R2;
end

t = V(:,end);
cam_centers(:,2) = [-t; 1];

Kb = K(:,:,2);

RIt(:,:,1) = Kb * R1 * [eye(3), t];
RIt(:,:,2) = Kb * R1 * [eye(3), -t];
RIt(:,:,3) = Kb * R2 * [eye(3), t];
RIt(:,:,4) = Kb * R2 * [eye(3), -t];

% temporary fix.
%cams(:,:,2) = Kb * RIt(:,:,1);
%return;
for i = 1:4
    cams_tmp(:,:,1) = cams(:,:,1);
    cams_tmp(:,:,2) = RIt(:,:,i);
    p(:,:,i) = reconstruct_point_cloud(cams_tmp, points2d);
    p(:,:,i) = p(:,:,i) / p(end,:,i);
end

Ma = [eye(3), zeros(3,1)];

for i = 1:4
    gp_first_camera = Ma*p(:,:,i) p(:,:,i);
    if i == 1
        gp = R1 * [eye(3), t] * p(:,:,i);
    elseif i == 2
        gp = R1 * [eye(3), -t] * p(:,:,i);
    elseif i == 3
        gp = R2 * [eye(3), t] * p(:,:,i);
    elseif i == 4
        gp = R2 * [eye(3), -t] * p(:,:,i);
    end

    if sign(gp(3)) == 1 && sign(gp_first_camera(3)) == 1
        cam(:,:,2) = RIt(:,:,i);
        if(i == 1 || i == 3)
            cam_centers(:,2) = [-t; 1];
        else
            cam_centers(:,2) = [t; 1];
        end
    end
end
