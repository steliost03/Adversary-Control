%state trajectory calculation for expanding controller
function [x,t_end] = covertExpandingTrajectory(A,B,G,w,umin,umax,x0,z0,xe,ue,ex,Kx,Q,q,q_x,L,l,c,c_chosen,adversaryControlEnabled,iterlimit,t_initial)
    
    if(not(adversaryControlEnabled))
         x = zeros(2,iterlimit);
    end
    u = zeros(1,iterlimit);
    
    x(:,t_initial) = x0;
    u(:,t_initial) = ue + Kx*(x(:,t_initial)-xe);
    iterations = t_initial;
    starting_sigma = switching_signal(1,x0,xe,A,B,G,w,umin,umax,c,z0,ex,Q,q,L,l,c_chosen,2);
    sigma = starting_sigma;
    while(and(iterations<iterlimit , sigma == 2))
        iterations = iterations + 1;
        x(:,iterations) = A*x(:,iterations-1) + B*(Kx*x(:,iterations-1)) - A*xe - B*(Kx*xe) + xe;
        u(:,iterations) = ue + Kx*(x(:,iterations)-xe);
        if(adversaryControlEnabled)
            newsigma = switching_signal(iterations,x(:,iterations),xe,A,B,G,w,umin,umax,c,z0,ex,Q,q,L,l,c_chosen,sigma);
            sigma = newsigma;
        end
    end
    
    if(not(adversaryControlEnabled))
         %resize array of states
        j_mark = iterlimit;
        for i=1:iterlimit
            if(and(abs(x(1,i))<0.0001,abs(x(2,i))<0.0001))
                j_mark = i;
            end
        end
        
        for i=j_mark:iterlimit
            x(:,i) = [];
        end 
    end
    t_end = length(x)+1;
end