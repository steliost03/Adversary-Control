%P,r : desired operation area
%G,w : alarm constraints area
function [xe,ue] = expanding_eqpoint(Asys,Bsys,P,r,G,w,umin,umax,strat)

    if(strat == '1')
        %STRAT 1 : farthest valid point from desired operation area

        %x(1,2) -> xe
        %x(3) -> ue

        n = 2; %can be generalized.

        xsym = sym('x',[2 1]);

        %set up linprog
        x0 = [-1;-1;0.5];
        A = [0 0 1
            0 0 -1];

        Gmod = [G zeros(size(G,1),1) ];
        A = [A;Gmod];

        b = [umax ; -umin];
        b = [b;w];

        beq = zeros(n,1);

        I = eye(n,n);
        AminusI = Asys-I;

        Aeq = [AminusI Bsys];

        options = optimoptions('fmincon','Display','off');
        xe_ue = fmincon(@(x)VD(x,xsym,P,r),x0,A,b,Aeq,beq,[],[],[],options);

        %output
        xe = zeros(2,1);
        xe(1) = xe_ue(1);
        xe(2) = xe_ue(2);

        ue = xe_ue(3);


    elseif(strat == '2')
        
        %STRAT 2 : random valid point
        
        %find all equilibrium points (solve linear system)
        Xe = zeros(2,10);
        i = 1;
        Alinsys = Asys - eye(size(Asys,1));
        for ue=umin:0.01:umax;

            b_linsys = -Bsys*ue;
            Xe(:,i) = linsolve(Alinsys,b_linsys);
            i = i + 1;
        end
        
         %exclude those points inside desired operation area,and outside of alarm
         %constraints

        Xe_valid = zeros(2,2);
        count = 0;

        for i=1:size(Xe,2)
            inside_desired = is_inside_conv_area(P,r,Xe(:,i));
            on_boundary = onBoundary(Xe(:,i),G,w);
            outside_alarm = and(not(is_inside_conv_area(G,w,Xe(:,i))),not(on_boundary));
            if(or(inside_desired,outside_alarm))
                continue
            elseif(on_boundary)
                count = count + 1;
                Xe_valid(:,count) = Xe(:,i);
            else
                count = count + 1;
                Xe_valid(:,count) = Xe(:,i); 
            end
        end
        
        randIndex = round(size(Xe_valid,2)*rand);
        if(randIndex == 0)
            randIndex = 1;
        end
        
        %choose random valid eqpoint
        Xe_chosen = Xe_valid(:,randIndex);
        xe = Xe_chosen;
        
        %find corresponding input ue
        identity = eye(size(Asys,1));
        blinsys = (identity - Asys)*xe;
        ue = linsolve(Bsys,blinsys);
        
       
    
    else
        
        %STRAT 3 : manual input
        disp '-----------------------------------'
        disp 'Input equilibrium point coordinates : ';
        dim = size(Asys,1);
        xe = zeros(dim,1);
        validpoint = false;
        
        while(validpoint == false)
            validpoint = true;
            for i=1:dim
                fprintf('\nDimension %d :',i);
                xe(i,1) = input('');
            end
            
            %check if selected point is valid
            inside_desired = is_inside_conv_area(P,r,xe);
            on_boundary = onBoundary(xe,G,w);
            outside_alarm = and(not(is_inside_conv_area(G,w,xe)),not(on_boundary));
            if(or(inside_desired,outside_alarm))
               disp 'The selected point is not valid.'
               disp ''
               validpoint = false;
            end
        end
        
        %find corresponding input ue
        identity = eye(size(Asys,1));
        blinsys = (identity - Asys)*xe;
        ue = linsolve(Bsys,blinsys);
        
        xe
        
    end
    
end



    