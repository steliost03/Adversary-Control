%Checks if sliding mode is needed (case where the next iteration will
%violate the hard state constraints) . Implements it if needed.
%G,w : hard state constraints
function u = sliding_mode(G,w,Asys,B,xcurrent,ucurrent,umax,umin)

    xnew = Asys*xcurrent + B*ucurrent;
    
    if(is_inside_conv_area(G,w,xnew))
        u = ucurrent;
    else
        
        %set up fmincon
        %objective function : argmin|B(unew-ucurrent)|,(variable:unew)
        %subject to
        %1: umin <= unew <= umax
        %2: G(A*xcurrent + B*unew) <= w
        
        Aeq = [];
        beq = [];
        ub = umax;
        lb = umin;
        
        A = G*B;
        b = w - G*(Asys*xcurrent);
        u0 = ucurrent;
        
        options = optimset('Display', 'off') ;
        unew = fmincon(@(unew)SM_OBJ(unew,B,ucurrent),u0,A,b,Aeq,beq,lb,ub,[],options);
        u = unew;
    end

end