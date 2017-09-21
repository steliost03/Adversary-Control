%checks if given point is inside the convex area defined by A,b
%Assuming convex area is of the form : {x : Ax <= b }
function output = is_inside_conv_area(A,b,point)

    dim = size(A,1);
    output = 1;
    for i=1:dim
        if(A(i,:)*point(:) - b(i) >= 0.05)
            output = 0;
            return;
        end
    end
end