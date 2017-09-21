function ustar_exists = feasibility_problem(x_current,U,Q,q,Asys,Bsys,G,w)
   
    umin = U(1);
    umax = U(2);
    
    for ustar = umin:0.01:umax
        x_value = Asys*x_current + Bsys*ustar;
        if(not(is_inside_conv_area(Q,q,x_value)))
            if(not(is_inside_conv_area(G,w,x_value)))
                continue;
            else
            ustar_exists = true;
            return;
            end
        end
    end
    
    ustar_exists = false;
    
end