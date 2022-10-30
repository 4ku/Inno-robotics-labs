% Plane s
p = [-5 11 0.5]';
v = [1 0 0]';
w = [4 2 -1]';
norm = null([v, w]');
shift = norm' * p;
s = [v w];

% Point g
g = [-10 -3 5]';

% Projection of the point g to the surface s
g_proj = (norm * shift) + s * inv(s' * s) * s' * g;

% The normal vector equal to distance of the point g
% from the plane s.
n = g - g_proj;

% Point symmetrical to g relative to the plane s.
g_star = g_proj - n;

drawPlane(norm, shift);
hold on
drawVector(norm, {'norm'});
drawVector(g, {'g'});
drawVector(g_proj, {'g_{proj}'});
drawVector(g_star, {'g_{star}'});

