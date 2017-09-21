%defines a hyperplane in H-representation : { Px = 1 }
%given adequate amount of points.
function P = defineHyperplane(points)
    
    pointCount = size(points,2);
    dim = size(points,1);
    
    %dim must be equal to pointCount
    
    A = transpose(points);
    b = ones(pointCount,1);
    
    P = linsolve(A,b);
    P = transpose(P);
    
end