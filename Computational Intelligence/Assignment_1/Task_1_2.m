% %  First plane
p1 = [0 1 0]';
v1 = [1 7 4]';
w1 = [2 8 5]';

% Compute surface normal
n1 = null([v1, w1]');
r0_1 = p1;
% So plane representation would be n1*(r-p1)

drawPlane(n1, n1' * r0_1);

% % Second plane
p2 = [-1 0 0]';
v2 = [-2 -2 1]';
w2 = [5 5 -5]';

% Compute surface normal
n2 = null([v2, w2]');
r0_2 = p2;
% So plane representation would be n2*(r-p2)

figure(2);
drawPlane(n2, n2' * r0_2);