% H = compute_rectification_matrix(points1, points2)
%
% Method: Determines the mapping H * points1 = points2
% 
% Input: points1, points2 of the form (4,n) 
%        n has to be at least 5
%
% Output:  H (4,4) matrix 
% 

function H = compute_rectification_matrix( points1, points2 )


% POINTS2 = P_prim
% POINTS1 = P

x = points1(1,:);
y = points1(2,:);
z = points1(3,:);
w = points1(4,:);
xp = points2(1,:);
yp = points2(2,:);
zp = points2(3,:);

len = size(points2,2);


w1 = [x', y', z', w', zeros(len,8), (-x .* xp)', (-y .* xp)', (-z .* xp)', (-w .*xp)'];
w2 = [zeros(len,4), x', y', z', w', zeros(len,4), (-x .* yp)', (-y .* yp)', (-z .* yp)', (-w .*yp)'];
w3 = [zeros(len,8), x', y', z', w', (-x .* zp)', (-y .* zp)', (-z .* zp)', (-w .*zp)'];

W = [w1;w2;w3];
[~, ~, V] = svd(W);
h = V(:,end);
H = reshape(h,4,4)';