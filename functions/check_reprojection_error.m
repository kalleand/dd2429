% function [error_average, error_max] = check_reprojection_error(data, cam, model)
%
% Method:   Evaluates average and maximum error 
%           between the reprojected image points (cam*model) and the 
%           given image points (data), i.e. data = cam * model 
%
%           We define the error as the Euclidean distance in 2D.
%
%           Requires that the number of cameras is C=2.
%           Let N be the number of points.
%
% Input:    points2d is a 3xNxC array, storing all image points.
%
%           cameras is a 3x4xC array, where cams(:,:,1) is the first and 
%           cameras(:,:,2) is the second camera matrix.
%
%           point3d 4xN matrix of all 3d points.
%       
% Output:   
%           The average error (error_average) and maximum error (error_max)
%      

function [error_average, error_max] = check_reprojection_error( points2d, cameras, point3d )


%------------------------------
% TODO: FILL IN THIS PART

error_max = 0;
error = 0;
C = size(cameras, 3);
N = size(point3d, 2);
for c = 1:C
    for n = 1:N
        p1 = homogeneous_to_cartesian(cameras(:,:,c) * point3d(:,n));
        p2 = homogeneous_to_cartesian(points2d(:,n,c));
        local_error = sqrt((p1(1)-p2(1))^2+(p1(2)-p2(2))^2);
        if local_error > error_max
            error_max = local_error;
        end
        error = error + local_error;
    end
end

error_average = error / (C*N);
