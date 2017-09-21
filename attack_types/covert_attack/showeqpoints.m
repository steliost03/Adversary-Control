%shows all valid equilibrium points for expanding controller
%G,w : alarm constraints
%P,r : desired operation area
%umin,umax : input constraints
function showeqpoints(A,B,G,w,P,r,umin,umax,axislimit)

    disp ''
    disp '--------------------------------------------------------'
    disp ''
    disp '1.Only show points on the boundary of alarm constraints.'
    disp '2.Only show points NOT on the boundary of alarm constraints.'
    disp '3.Show all valid equilibrium points.'
    
    showselect = input('');
    
    if(showselect == 1)
        FORCE_BOUNDARY = true;
        EXCLUDE_BOUNDARY = false;
    elseif(showselect == 2)
        FORCE_BOUNDARY = false;
        EXCLUDE_BOUNDARY = true;
    elseif(showselect == 3)
        FORCE_BOUNDARY = false;
        EXCLUDE_BOUNDARY = false;
    end
    
    %find all equilibrium points (solve linear system)
    Xe = zeros(2,10);
    i = 1;
    Alinsys = A - eye(size(A,1));
    for ue=umin:0.01:umax;

        b_linsys = -B*ue;
        Xe(:,i) = linsolve(Alinsys,b_linsys);
        i = i + 1;

    end

    %exclude those points inside desired operation area,and outside of alarm
    %constraints

    Xe_boundary = zeros(2,2); %all eqpoints ON boundary.
    Xe_other = zeros(2,2); %all valid eqpoints except those on boundary.
    Xe_final = zeros(2,2); %the eqpoints that will be drawn.
    other_count = 0;
    boundary_count = 0;

    for i=1:size(Xe,2)
        inside_desired = is_inside_conv_area(P,r,Xe(:,i));
        on_boundary = onBoundary(Xe(:,i),G,w);
        outside_alarm = and(not(is_inside_conv_area(G,w,Xe(:,i))),not(on_boundary));
        if(or(inside_desired,outside_alarm))
            continue
        elseif(on_boundary)
            boundary_count = boundary_count + 1;
            Xe_boundary(:,boundary_count) = Xe(:,i);
        else
            other_count = other_count + 1;
            Xe_other(:,other_count) = Xe(:,i); 
        end
    end
 
    %DRAWING
    %desired operation area
    [x1_D,x2_D] = define_polyhedral(1,P,r,0);
    %alarm constraints area
    [x1_A,x2_A] = define_polyhedral(1,G,w,0);
     
    figure;
    plot(x1_D,x2_D,'g');
    hold on;
    plot(x1_A,x2_A,'r');
    hold on;
    if(FORCE_BOUNDARY)
        scatter(Xe_boundary(1,:),Xe_boundary(2,:));
        legend('Desired Operation','Alarm constraints','Valid eq. points');
    elseif(EXCLUDE_BOUNDARY)
        plot(Xe_other(1,:),Xe_other(2,:),'c.');
        legend('Desired Operation','Alarm constraints','Valid eq. points');
    else
        scatter(Xe_boundary(1,:),Xe_boundary(2,:));
        hold on;
        plot(Xe_other(1,:),Xe_other(2,:),'c.');
        legend('Desired Operation','Alarm constraints','Boundary eqpoints', ...
        'Other eqpoints');
    end
    title('Equilibrium Points')
     
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
     
end