% function [norm_mat] = get_normalization_matrices(data);   
%
% Method: get all normalization matrices.  
%         It is: point_norm = norm_matrix * point. The norm_points 
%         have centroid 0 and average distance = sqrt(2))
% 
% Input: data (3*m,n) (not normalized) the data may have NaN values 
%        for m cameras and n points 
%        
% Output: norm_mat is a 3*m,3 matrix, which consists of all 
%         normalization matrices matrices, i.e. norm_mat(1:3,:) is the 
%         matrix for camera 1 
%

function [norm_mat] = get_normalization_matrices(data)   


% get Info 
am_cams = size(data,1)/3;
am_points = size(data,2);

for cam = 1:am_cams
    
    % Evaluate centroid for image.
    c_x = 0;
    c_y = 0;
    points = 0;
    for p = 1:am_points
        p_x = data(cam*3-2,p);
        p_y = data(cam*3-1,p);
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
        p_x = data(cam*3-2,p);
        p_y = data(cam*3-1,p);
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
    norm_mat(cam*3-2:cam*3,:) = N;
end
