p1 = [0 0 0]';
v1 = [7 4 3]';
w1 = [9 11 13]';
n1 = null([v1, w1]');

p2 = [0 0 0]';
v2 = [0 1 2]';
w2 = [3 4 1]';
n1 = null([v2, w2]');

if (det([v1 w1 v2]) > 0) || (det([v1 w1 w2]) > 0)
    disp('Planes intersect')
else
    if isequal(n1' * p1, n2' * p2) %if shifts of planes are equal
        disp('Planes are the same')
    else
        disp('Planes are parallel')
    end
end 

drawSpan([v1 w1], 'r');
hold on
drawSpan([v2 w2], 'b');