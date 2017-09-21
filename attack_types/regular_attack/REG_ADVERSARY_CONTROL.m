%{
Copyright (C) 2017 Stylianos Tsiakalos
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
%}


function REG_ADVERSARY_CONTROL()

    disp 'ADVERSARY CONTROL';
    disp 'In a 2x2 SISO system under linear constraints';

    disp ''
    PAPER_DEFAULTS_inp = input('Use test example? <y/n>','s');
    if(PAPER_DEFAULTS_inp == 'y' || PAPER_DEFAULTS_inp == 'Y')
        PAPER_DEFAULTS = true;
    else
        PAPER_DEFAULTS = false;
    end
    cons_moves_inp = input('Enable consecutive moves? <y/n>','s');
    if(cons_moves_inp == 'y' || cons_moves_inp == 'Y') 
        nc = input('Enter contractive controller move number:');
        ne = input('Enter expanding controller move number:');
        turnCount = input('Enter number of turns of the game:');
        uncertaintyEnabled = input('Include measurement uncertainties for expanding controller? Enter 0 for no,any other number for yes:');
        if(uncertaintyEnabled)
            delta = input('Enter upper bound for measurement error :');
            seed = 0;
        else
            delta = 0;
            seed = 0;
        end
        CONSECUTIVE_MOVES = true;
    else
        CONSECUTIVE_MOVES = false;
        turnCount = 1;
        uncertaintyEnabled = false;
        seed = 0;
        delta = 0;
    end

    if(PAPER_DEFAULTS)
        A = [0.9 0.2;0 0.9];
        B = [-0.3;-0.2];
        x0 = [3;-0.5];
        G = [-1 -0.1;-0.5 0.9;0.9 0.4;-0.2 -1];
        w = [1.6;1.7;4.1;2.7];
        umin = -0.8;
        umax = 1.2;
        AXISLIMIT = 8;
        IS_3D = false;
        IS_N_DIM = false;
    end

    if(not(PAPER_DEFAULTS))
            disp 'Enter system matrix (A) '
            A = input('');
            disp 'Enter input matrix (B) '
            B = input('');
            disp 'Enter initial state (as a vector)'
            x0 = input('');
            disp 'Assuming linear state constr : Gx <= w'
            G = input('Enter G matrix : ');
            w = input('Enter w vector : ');
            disp 'Assuming linear input constr : umin<=u<=umax'
            umin = input('Enter umin : ');
            umax = input('Enter umax : ');
            AXISLIMIT = input('Enter limit of axes : ');
            if(size(A,1) == 3)
                IS_3D = true;
            else
                IS_3D = false;
            end
            if(size(A,2) > 3)
                IS_N_DIM = true;
            else
                IS_N_DIM = false;
            end
    end
    
    tic;
    
    %INITIALIZATION
    v = [umax ; -umin];
    if(not(PAPER_DEFAULTS))
        [Kinitial,e] = kon_proposition(A,B,G,w,v);
    %
    else
         %[0.1761 0.6426] : paper result.
         %[0.1787 0.6480] : custom result for paper values
         Kinitial = [0.1787 0.6480];
         e = 0.8443;
    end
  
    %
    F = [Kinitial ; -Kinitial];

    P = [G ; F];
    r = [w ; v];

    xcon = zeros(2,100);
    ucon = zeros(1,100);
    xexp = zeros(2,100);
    uexp = zeros(1,100);
    xgeneral = zeros(2,200*turnCount);
    xcon(:,1) = x0;
    xexp(:,1) = x0;
    xgeneral(:,1) = x0;

    if(not(CONSECUTIVE_MOVES))
        %CONTRACTIVE ONLY
        iterations = 1;
        ucon(1) = contractive_algorithm(A,B,P,r,v,x0,umax,umin,Kinitial);
        while(and( (iterations <500), not(and(abs(xcon(1,iterations))<0.001,abs(xcon(2,iterations))<0.001))))
            iterations = iterations + 1;
            xcon(:,iterations) = A*xcon(:,iterations-1) + B*ucon(iterations-1);
            ucon(iterations) = contractive_algorithm(A,B,P,r,v,xcon(:,iterations),umax,umin,Kinitial);
        end



        %EXPANDING ONLY
        iterations = 1;
        seed = seed + 1;
        uexp(1) = expanding_algorithm(A,B,P,r,G,w,x0,umax,umin,uncertaintyEnabled,delta,seed);
        for i = 1:500

            xexp(:,iterations+1) = A*xexp(:,iterations) + B*uexp(iterations);
            seed = seed + 1;
            uexp(iterations+1) = expanding_algorithm(A,B,P,r,G,w,xexp(:,iterations+1),umax,umin,uncertaintyEnabled,delta,seed);
            iterations = iterations + 1;

        end

    else
        currentTurnCount = 1;
        general_counter = 1;
        xcon_relinquish_indices = zeros(turnCount,1);
        xexp_relinquish_indices = zeros(turnCount+1,1);
        xexp_relinquish_indices(1) = 0;
        switch_counter_con = 1;
        switch_counter_exp = 2;

        while(currentTurnCount <= turnCount)

            %contractive
            ucon(1) = contractive_algorithm(A,B,P,r,v,x0,umax,umin,Kinitial);
            iterations = 1;
            while(iterations<nc)
                iterations = iterations + 1;
                general_counter = general_counter + 1;
                xgeneral(:,general_counter) = A*xgeneral(:,general_counter-1) + B*ucon(iterations-1);
                ucon(iterations) = contractive_algorithm(A,B,P,r,v,xgeneral(:,general_counter),umax,umin,Kinitial);
            end
            xcon_relinquish_indices(switch_counter_con) = general_counter;

            %expanding
            x0_exp = xgeneral(:,general_counter);
            seed = seed + 1;
            uexp(1) = expanding_algorithm(A,B,P,r,G,w,x0_exp,umax,umin,uncertaintyEnabled,delta,seed);
            iterations = 1;
            while(iterations <= ne)
                iterations = iterations + 1;
                general_counter = general_counter + 1;
                xgeneral(:,general_counter) = A*xgeneral(:,general_counter-1) + B*uexp(iterations-1);
                seed = seed + 1;
                uexp(iterations) = expanding_algorithm(A,B,P,r,G,w,xgeneral(:,general_counter),umax,umin,uncertaintyEnabled,delta,seed);
            end

            if(not(currentTurnCount==turnCount))
             xexp_relinquish_indices(switch_counter_exp) = general_counter;
            end

            currentTurnCount = currentTurnCount + 1;
            switch_counter_con = switch_counter_con + 1;
            switch_counter_exp = switch_counter_exp + 1;

        end 
    end

    if(CONSECUTIVE_MOVES)
        xexp_relinquish_indices(turnCount+1) = turnCount*(nc+ne);
    end

    %resize arrays
    %mark : the point in which the controller stops.
    if(not(CONSECUTIVE_MOVES))
    xcon_mark = 100;
    ucon_mark = 100;
    xexp_mark = 100;
    uexp_mark = 100;
    for i=1:100
        if (and(abs(xcon(1,i)) < 0.001,abs(xcon(2,i)) < 0.001))
            xcon_mark = i;
            break;
        end
    end
    for i=1:100
        if (abs(ucon(1,i))<0.001)
            ucon_mark = i;
            break;
        end
    end
    for i=1:100
        if (and(abs(xexp(1,i)) <0.001,abs(xexp(2,i))<0.001))
            xexp_mark = i;
            break;
        end
    end
    for i=1:100
        if (abs(uexp(1,i))<0.001)
            uexp_mark = i;
            break;
        end
    end

    for i=ucon_mark+1:100
        ucon(:,ucon_mark+1) = [];
    end

    for i=uexp_mark+1:100
         uexp(:,uexp_mark+1) = [];
    end

    for i=xcon_mark+1:100
         xcon(:,xcon_mark+1) = [];
    end

    for i=xexp_mark+1:100
         xexp(:,xexp_mark+1) = [];
    end

    end

    %
    if(CONSECUTIVE_MOVES)
        xcon = zeros(2,turnCount*(nc+ne));
        xexp = zeros(2,turnCount*(nc+ne));
        xcon_counter = 1;
        xexp_counter = 1;
        for j=1:turnCount
            for i=xexp_relinquish_indices(j)+1:xcon_relinquish_indices(j)
                xcon(:,xcon_counter) = xgeneral(:,i);
                xcon_counter = xcon_counter + 1;
            end
            for i=xcon_relinquish_indices(j)+1:xexp_relinquish_indices(j+1)
                xexp(:,xexp_counter) = xgeneral(:,i);
                xexp_counter = xexp_counter + 1;
            end
        end
    end

    %DRAWING%

    %---state constraints---
    [state_constraints(1,:),state_constraints(2,:)] = define_polyhedral(1,G,w,0);

    %---input constraints---
    constraint_lines_inp = zeros(2,2,2);

    constraint_lines_inp(:,:,1) = [1000 -1000;(umax-1000*Kinitial(1))/Kinitial(2) (umax+1000*Kinitial(1))/Kinitial(2)];
    constraint_lines_inp(:,:,2) = [1000 -1000;(umin-1000*Kinitial(1))/Kinitial(2) (umin+1000*Kinitial(1))/Kinitial(2)];

    first_constraint_line = constraint_lines_inp(:,:,1);
    second_constraint_line = constraint_lines_inp(:,:,2);

    inp_far_plane_line1_point1 = point_perp(first_constraint_line,constraint_lines_inp(:,1,1),-1000);
    inp_far_plane_line1_point2 = point_perp(first_constraint_line,constraint_lines_inp(:,2,1),-1000);

    inp_far_plane_line2_point1 = point_perp(second_constraint_line,constraint_lines_inp(:,1,2),1000);
    inp_far_plane_line2_point2 = point_perp(second_constraint_line,constraint_lines_inp(:,2,2),1000);

    %separate x's and y's (separate arguments in polybool)
    %line 1 plane
    line1_plane_x = [inp_far_plane_line1_point1(1,1),constraint_lines_inp(1,1,1),constraint_lines_inp(1,2,1),inp_far_plane_line1_point2(1,1)];
    line1_plane_y = [inp_far_plane_line1_point1(2,1),constraint_lines_inp(2,1,1),constraint_lines_inp(2,2,1),inp_far_plane_line1_point2(2,1)];

    %line 2 plane
    line2_plane_x = [inp_far_plane_line2_point1(1,1),constraint_lines_inp(1,1,2),constraint_lines_inp(1,2,2),inp_far_plane_line2_point2(1,1)];
    line2_plane_y = [inp_far_plane_line2_point1(2,1),constraint_lines_inp(2,1,2),constraint_lines_inp(2,2,2),inp_far_plane_line2_point2(2,1)];

    [xpolyhedral,ypolyhedral] = polybool('intersection',line1_plane_x,line1_plane_y,line2_plane_x,line2_plane_y);


    %%DRAWING%%

    %draw input constraints
    figure;
    plot(xpolyhedral,ypolyhedral,'g');
    hold on;


    %draw state constraints
    plot(state_constraints(1,:),state_constraints(2,:),'k');
    hold on;

    %tranform in a form suitable for plotting 
    if(CONSECUTIVE_MOVES)
        counter = 1;
        for i=1:size(xcon,2)
            if(not(and(abs(xcon(1,i))<0.001,abs(xcon(2,i))<0.001)))
                xcon_plot(:,counter) = xcon(:,i);
                counter = counter + 1;
            end
        end

        counter = 1;
        for i=1:size(xexp,2)
            if(not(and(abs(xexp(1,i))<0.001,abs(xexp(2,i))<0.001)))
                xexp_plot(:,counter) = xexp(:,i);
                counter = counter + 1;
            end
        end
    end

    %draw state progression
    if(CONSECUTIVE_MOVES)
        plot(xcon_plot(1,:),xcon_plot(2,:),'b.');
        hold on;
        plot(xexp_plot(1,:),xexp_plot(2,:),'r.');
        hold on;
    else
        plot(xcon(1,:),xcon(2,:),'b.-');
        hold on;
        plot(xexp(1,:),xexp(2,:),'r.-');
        hold on;
    end

    %show initial state
    scatter(x0(1),x0(2),'k');


    %draw axis lines
    axis square;
    title('ADVERSARY CONTROL');
    if(not(CONSECUTIVE_MOVES))
    legend('input constraints','state constraints','contractive only','expanding only','initial state');
    else
    legend('input constraints','state constraints','contractive','expanding','initial state');
    end

    xL = xlim;
    yL = ylim;
    line([0 0], yL);  %x-axis
    line(xL, [0 0]);  %y-axis
    xlim([-AXISLIMIT AXISLIMIT]);
    ylim([-AXISLIMIT AXISLIMIT]);
    xlabel('x1');
    ylabel('x2');
    
    toc;
 
end