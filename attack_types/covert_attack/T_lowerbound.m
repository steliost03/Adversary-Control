function output = T_lowerbound(c,z0,ex,Q,q,J,j)
  
    %check if zero is in the boundary of M_z or in its interior
    onBoundary = false;
    zeroPos = [0;0];
    for i=1:(size(Q,1))
        if( abs((Q(i,:)*zeroPos(:)) - q(i)) < 0.001)
            onBoundary = true;
        end
    end
    
    if(onBoundary)
        V_M = lyapunov_value(2,J,j,z0);
    else
        V_M = lyapunov_value(1,Q,q,z0); 
    end
    
    %calculate lower bound of T
    temp = c / V_M;
    numerator = log(temp);
    denominator = log(ex);
    
    output = numerator/denominator;
      
end