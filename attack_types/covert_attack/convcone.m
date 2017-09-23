%implements the convex cone strategy for expanding controller
%also handles drawing (instead of the main function)
function convcone(A,B,x0,G,w,P,r,umin,umax,xe,ue,axislimit)
 
  
     %ANY DIMENSIONS
    %desired operation area
    D = Polyhedron('A',P,'b',r);
    %alarm constraints area
    Alarm = Polyhedron('A',G,'b',w);
    %%% (using mpt for the representation and drawing)

    %find suitable facet for the equilibrium point
        
  
        %define all half-spaces from facets of desired operation
        facetCount = size(P,1);
        for i=1:facetCount
            P_v = P(i,:);
            r_v = r(i);
            if(is_inside_conv_area(-P_v,-r_v,xe))
                [C,c,facetLimitPoints] = getC_convhull(G,w,P,r,x0,i);
                if(is_inside_conv_area(C,c,xe))
                    break;
                end
            end
        end

       Cone = Polyhedron('A',C,'b',c);
       Cone.minHRep();
       inequalities = Cone.H;
       dim = size(A,1);
       inequalityCount = size(inequalities,1);

       C_optim = zeros(inequalityCount,dim);
       c_optim = zeros(inequalityCount,1);

       for i=1:inequalityCount
           for j=1:dim
               C_optim(i,j) = inequalities(i,j);
           end
           c_optim(i) = inequalities(i,dim+1);
       end

       %use theorem to make cone contractive
       %change of variable : z = x - xe ,where xe: chosen eq point
       onboundary = onBoundary(xe,C_optim,c_optim);
       if(not(onboundary))
           %if xe lies inside of (C,c)
           umin_new = umin - ue;
           umax_new = umax - ue;
           c_optim_z = c_optim - C_optim*xe;
           v = [umax_new; -umin_new];
           [K,e] = kon_theorem(A,B,C_optim,c_optim_z,v);
           
           
          
       else
           %if xe lies in the boundary of (C,c)
           %enlarge (C,c) so that xe is no longer on the boundary
            %find the inequality that needs to be modified
           CC_facetCount = size(C_optim,1);
           for i=1:CC_facetCount
               if(onBoundary(xe,C_optim(i,:),c_optim(i)))
                   c_optim(i) = c_optim(i) + 10;
               end
           end
           umin_new = umin - ue;
           umax_new = umax - ue;
           c_optim_z = c_optim - C_optim*xe;
           v = [umax_new; -umin_new];
           [K,e] = kon_theorem(A,B,C_optim,c_optim_z,v);
           %[K,e] = kon_theorem_modified(A,B,C_optim,c_optim_z,umax_new,umin_new,ue);
          
          
           %
       end

       %calculate trajectory
       [trajectory,stepCount] = obtainTrajectory(A,B,K,x0,xe,ue,1000);
   

    fprintf('\nReached equilibrium point in %d',stepCount);
    fprintf(' steps.\n');

    %FIGURES
    %
    figure;
    plot(Alarm,'color','w','edgecolor','r','alpha',0);
    hold on;
    plot(D,'color','w','edgecolor','g','alpha',0);
    hold on;
    plot(Cone,'color','w','edgecolor','k','alpha',0);
    hold on;
    plot(x0(1),x0(2),'k*');
    hold on;
    plot(xe(1),xe(2),'b*');
    hold on;
    plot(facetLimitPoints(1,:),facetLimitPoints(2,:),'c*');
    hold on;
    plot(trajectory(1,:),trajectory(2,:),'m.-');
    legend('Alarm constraints','Desired Operation','Convex cone','Initial point','Chosen eqpoint','Facet Limit Points','Trajectory');

    %draw axis lines
    axis square;
    xlim([-axislimit axislimit]);
    ylim([-axislimit axislimit]);
    xL = xlim;
    yL = ylim;
    line([0 0], yL);  %x-axis
    line(xL, [0 0]);  %y-axis
    xlabel('x1');
    ylabel('x2');
    title('Convex cone method');

end
