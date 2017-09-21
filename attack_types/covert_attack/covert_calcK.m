%q is returned with respect to z = x - xe.
function [e,K,Q,q] = covert_calcK(chosenFacetIndex,A,B,x0,xe,P,r,G,w,umin,umax,ue)
    
        %determine P_v_inv
        z0 = x0 - xe;
        P_v_inv = P(chosenFacetIndex,:);
        r_v = r(chosenFacetIndex);
        
        %determine c0
        r_v_new = -r_v + P_v_inv*xe;
        c0 = (-P_v_inv*z0) /r_v_new;

        %determine Q,q
        wnew = w - G*xe; 
        Q = [G ; -P_v_inv];
        q = [wnew ; c0*r_v_new];

        %express M_z in terms of x (M : Q* x <= q_X )
        q_x = q+Q*xe;

        onBoundary = false;
        for i=1:(size(Q,1))
            if( abs((Q(i,:)*xe(:)) - q_x(i)) < 0.01)
                onBoundary = true;
            end
        end
        if(onBoundary)
         [Kx,ex] = kon_theorem_modified(A,B,Q,q,umax,umin,ue);
        else
         umax_new = umax - ue;
         umin_new = umin - ue;
         vnew = [umax_new ; -umin_new];
         [Kx,ex] = kon_theorem(A,B,Q,q,vnew);
        end
        
        e = ex;
        K = Kx;
end

    