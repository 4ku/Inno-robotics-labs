p = [2 2 2]';
v = [-2 3 3]';
w = [1 1 -5]';

% Compute surface normal. It also would be vector of line l
% passing through the origin.
n = null([v, w]');

% point g
g = [-10 -3 5]';

% Projection of the point g to line l
g_proj = n * inv(n' * n) * n' * g;

drawPlane(n, n' * p);
hold on
drawVector(n, {'n'});
drawVector(g, {'g'});
drawVector(g_proj, {'g_{proj}'});

