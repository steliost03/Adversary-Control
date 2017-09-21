%expandingData = [umin,umax,c,z0,ex,Q,q,L,l,c_chosen,xe,ue,q_x];
function [xcon,xexp,nc,ne,con_exists] = covertAdversaryAlgorithm(A,B,G,w,P,r,Kc1,Kc2,Kx,umin,umax,c,z0,ex,Q,q,L,l,c_chosen,xe,ue,q_x,x0,iterlimit)

    xcon = zeros(2,iterlimit);
    xexp = zeros(2,iterlimit);
    
    con_exists = false;
    
    nc = 0;
    ne = 0;
    
    [xexp_curr,t_expEnd] = covertExpandingTrajectory(A,B,G,w,umin,umax,x0,z0,xe,ue,ex,Kx,Q,q,q_x,L,l,c,c_chosen,true,iterlimit,1);

    for i=1:t_expEnd-1
        xexp(:,i) = xexp_curr(:,i);
        ne = ne + 1;
    end
    tend = t_expEnd;
    
    while(tend-1<iterlimit)
        con_exists = true;
        [xcon_curr,t_conEnd] = covertContractive(A,B,G,w,Kc1,Kc2,xexp(:,tend-1),P,r,true,iterlimit,t_expEnd,umin,umax,c,z0,ex,xe,Q,q,L,l,c_chosen);
        for i=t_expEnd:t_conEnd-1
            xcon(:,i) = xcon_curr(:,i);
            nc = nc + 1;
        end
        tend = t_conEnd;
        
        if(tend-1>iterlimit)
            break
        end
        [xexp_curr,t_expEnd] = covertExpandingTrajectory(A,B,G,w,umin,umax,xcon(:,tend-1),z0,xe,ue,ex,Kx,Q,q,q_x,L,l,c,c_chosen,true,iterlimit,t_conEnd);
        for i=t_conEnd:t_expEnd-1
            xexp(:,i) = xexp_curr(:,i);
            ne = ne + 1;
        end
        tend = t_expEnd;
    end
    
end