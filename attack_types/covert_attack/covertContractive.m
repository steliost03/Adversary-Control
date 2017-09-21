%extraVars : only passed in adversary control mode , otherwise pass 0.
%extraVars = [umin,umax,c,z0,ex,Q,q,L,l,c_chosen]
function [x,t_end] = covertContractive(A,B,G,w,Kc1,Kc2,x0,P,r,adversaryControlEnabled,iterlimit,t_initial,umin,umax,c,z0,ex,xe,Q,q,L,l,c_chosen)
    
    if(not(adversaryControlEnabled))
        x = zeros(2,iterlimit);
    end
    u = zeros(1,iterlimit);
    
    x(:,t_initial) = x0;
    if(is_inside_conv_area(P,r,x0))
        Kcurrent = Kc1;
    else
        Kcurrent = Kc2;
    end
    u(t_initial) = Kcurrent*x(:,t_initial);
    
    iterations = t_initial;
    sigma = 1;
    while( (iterations <iterlimit) && not(abs(x(1,iterations))<0.001) && not(abs(x(2,iterations))<0.001) && (sigma==1))
        
        iterations = iterations + 1;
        x(:,iterations) = A*x(:,iterations-1)+B*u(iterations-1);
        
        if(is_inside_conv_area(P,r,x(:,iterations)))
            Kcurrent = Kc1;
        else
            Kcurrent = Kc2;
        end
        
        u(iterations) = Kcurrent*x(:,iterations);
       
        if(adversaryControlEnabled)
            newsigma = switching_signal(iterations,x(:,iterations),xe,A,B,G,w,umin,umax,c,z0,ex,Q,q,L,l,c_chosen,sigma);
            sigma = newsigma;
        end
    end
    
    
    if(not(adversaryControlEnabled))
        %resize array of states
        j_mark = 0;
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