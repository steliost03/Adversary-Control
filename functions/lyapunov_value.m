%secondary function used, to calculate value of polyhedral lyapunov function.
%type 1 : max(Px/r}
%type 2: max{Px/r , 0}

function output = lyapunov_value(type,P,r,x)
    
    temparray = zeros(1,length(r));
    Px = P*x;
    if(type ==2)
        for i = 1:length(r)
            if( (Px(i)/r(i))>0)
                temparray(i) = Px(i)/r(i);
            else
                temparray(i) = 0;
            end
        end
    elseif(type == 1)
        for i=1:length(r)
            temparray(i) = Px(i) / r(i);
        end
    end
        
    
    output = max(temparray);
end