%P,r -> desired operation area ( Px <= r)
%A -> open loop system matrix , B-> system input matrix
%delta -> maximum measurement error
function u = expanding_algorithm(A,B,P,r,G,w,xcurrent,umax,umin,uncertaintyEnabled,delta,seed)
    
    xnew_umin = A*xcurrent + B*umin;
    xnew_umax = A*xcurrent + B*umax;
    
    %lyapunov values for umin and umax
    Umin = lyapunov_value(2,P,r,xnew_umin);
    Umax = lyapunov_value(2,P,r,xnew_umax);
    
    if(uncertaintyEnabled)
        dim = size(A,1);
        u = uncertaintyAlgorithm(dim,P,r,xcurrent,umax,umin,delta,seed);
    else
    
        if(Umin >= Umax)
            u = umin;
        else
            u = umax;
        end
    
    end
    
    u = sliding_mode(G,w,A,B,xcurrent,u,umax,umin); 
    
end