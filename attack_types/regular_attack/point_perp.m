% offsets point x a distance r perpendicular to line
% the sign of r defines the direction of the offset
% line = [p1 ; p2] = [p1x p2x ; p1y p2y]
% x = [xx ; xy] a point on line
% r is how far x_new will be from line
% the sign of r determines if x_new will be on the left or right of the line

function x_new = point_perp(line, x, r)

th = atan2( (line(2,2)-line(2,1)) , (line(1,2)-line(1,1)) );

if th >= 0
    x_new(1,1) = x(1) + r*sin(th);
    x_new(2,1) = x(2) - r*cos(th);   
else   
    x_new(1,1) = x(1) + r*cos(pi/2 - th);
    x_new(2,1) = x(2) - r*sin(pi/2 - th);
end

