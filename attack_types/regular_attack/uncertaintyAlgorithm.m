%implements the procedure regarding uncertainties in measurement.
%dim : dimension of A matrix (the number of states).
function u = uncertaintyAlgorithm(dim,P,r,xcurrent,umax,umin,delta,seed)
    
    clk = clock + 10*seed;
    rng(round(10000*clk(6)));
    
    %step 1: implement measurement error (random with upper bound = delta)
    %xmeasured = xcurrent +- delta
    xmeasured = xcurrent + delta*(2*rand - 1);
    
   
    %step 2: grid the state space
    A1 = 0; %counts toward umin
    A2 = 0; %counts toward umax
    
    
    for i=1:dim
        x_lower = xmeasured(i) - 2*delta;
        x_upper = xmeasured(i) + 2*delta;
    end
    
    for i=1:dim
        xiter = transpose(linspace(x_lower,x_upper,500));
    end
     
    
    for i=1:length(xiter)
        xgrid = xiter(i);
        Vumin = lyapunov_value(2,P,r,xgrid);
        Vumax = lyapunov_value(2,P,r,xgrid);
        if(Vumin >= Vumax)
            A1 = A1 + 1;
        else
            A2 = A2 + 1;
        end
    end
    
    
    if(A1>=A2)
        u = umin;
    else
        u = umax;
    end
    
end