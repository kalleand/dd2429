% H = det_homographies(points1, points2)
%
% Method: Determines the mapping H * points1 = points2
% 
% Input:  points1, points2 are of the form (3,n) with 
%         n is the amount of points 
%         The points should be normalized for 
%         better performance 
% 
% Output: H (3,3) matrix 
%

function H = det_homographies(points1, points2)

%------------------------------
%
% Inses enkelt.
%
%------------------------------

am_points = size(points1, 2);

alpha = [points1(1,:);
    points1(2,:);
    ones(1,am_points);
    zeros(1,am_points);
    zeros(1,am_points);
    zeros(1,am_points); 
    -(points1(1,:)).*points2(1,:); 
    -(points1(2,:)).*points2(1,:);
    -(points2(1,:))].';

beta = [zeros(1,am_points);
    zeros(1,am_points);
    zeros(1,am_points);
    points1(1,:);
    points1(2,:);
    ones(1,am_points);
    -(points1(1,:)).*points2(2,:);
    -(points1(2,:)).*points2(2,:);
    -(points2(2,:))].';
    

Q = [alpha; beta];

[U,S,V] = svd(Q);

h = V(:,end);

H = [h(1:3).'; h(4:6).'; h(7:9).'];
