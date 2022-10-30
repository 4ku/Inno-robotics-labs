V = [3 1 1; 6 2 2; -9 -3 -3];

% Task 2.1
basis = null(V);
norm = null(basis');
drawPlane(norm, 0);
hold on

% Task 2.2
g_proj = basis * inv(basis' * basis) * basis' * g;
n = g - g_proj;

drawVector(g, {'g'});
drawVector(g_proj, {'g_{proj}'});
drawVector(n, {'n'});

% Prove that n and g_proj perpendicular
if n' * g_proj < 10^-10
    disp("Vectors are perpendicular")
else
    disp("Vectors are not perpendicular")
end

% Task 2.3
% Recovering
g_recovered = n + g_proj;
if isequal(g, g_recovered)
    disp("g and g_recovered are equal")
else
    disp("g and g_recovered are not equal")
end    

