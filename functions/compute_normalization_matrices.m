% Method:   compute all normalization matrices.  
%           It is: point_norm = norm_matrix * point. The norm_points 
%           have centroid 0 and average distance = sqrt(2))
% 
%           Let N be the number of points and C the number of cameras.
%
% Input:    points2d is a 3xNxC array. Stores un-normalized homogeneous
%           coordinates for points in 2D. The data may have NaN values.
%        
% Output:   norm_mat is a 3x3xC array. Stores the normalization matrices
%           for all cameras, i.e. norm_mat(:,:,c) is the normalization
%           matrix for camera c.

function norm_mat = compute_normalization_matrices( points2d )

%-------------------------
% TODO: FILL IN THIS PART

% get Info 
am_cams = size(points2d,3);
am_points = size(points2d,2);

% initialize the norm_matrix
norm_mat = zeros(3,3,am_cams);

for cam = 1:am_cams
    
    % Evaluate centroid for image.
    c_x = 0;
    c_y = 0;
    points = 0;
    for p = 1:am_points
        p_x = points2d(1,p,cam);
        p_y = points2d(2,p,cam);
        if not(isnan(p_x))
            c_x = c_x + p_x;
            c_y = c_y + p_y;
            points = points + 1;
        end
    end
    c_x = c_x / points;
    c_y = c_y / points;
    
    % Evaluate mean distance for the points in the image.
    distance = 0;
    for p = 1:am_points
        p_x = points2d(1,p,cam);
        p_y = points2d(2,p,cam);
        if not(isnan(p_x))
            distance = distance + sqrt((p_x - c_x)^2 + (p_y - c_y)^2);
        end
    end
    distance = distance / points;
    
    % Construct the resulting norm_mat that corresponds to point cam.
    sqrt2 = sqrt(2);
    N = [sqrt2/distance, 0, -1*sqrt2*c_x/distance;
        0, sqrt2/distance, -1*sqrt2*c_y/distance;
        0, 0, 1];
    norm_mat(:,:,cam) = N;
end