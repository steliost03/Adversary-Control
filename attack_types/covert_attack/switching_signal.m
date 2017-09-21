%k : current time instance.
function sigma = switching_signal(k,x_current,xe,Asys,Bsys,G,w,umin,umax,c,z0,ex,Q,q,L,l,c_chosen,previous_sigma)
    
    
    T = T_lowerbound(c,z0,ex,Q,q,L,l);
    
    %find Mc and Mc'(Mcnew) expressed in terms of x
    q_Mc = c*q;
    q_Mcnew = c_chosen*q;
    
    q_x_Mc = q_Mc + Q*xe;
    q_x_Mcnew = q_Mcnew + Q*xe;
    
    %
    U = [umin umax];
    ustar_exists = feasibility_problem(x_current,U,Q,q_x_Mc,Asys,Bsys,G,w);
    x_in_desired_area = is_inside_conv_area(Q,q_x_Mcnew,x_current);
    if(not(c_chosen == c))
        
        if(or(k<=T,ustar_exists))
            sigma = 2;
       
        elseif(x_in_desired_area && not(ustar_exists))
            sigma = 1;
          
        else
            sigma = previous_sigma;

        end
        
    else
        if(or(k <= T,ustar_exists))
            sigma = 2;
        else
            sigma = 1;
        end
    end   
end