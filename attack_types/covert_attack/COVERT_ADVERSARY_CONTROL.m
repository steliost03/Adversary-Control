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


%Covert Attack on a Discrete-Time System with Limited Use of the Available
%Disruption Resources

function COVERT_ADVERSARY_CONTROL()

    addpath(genpath('../functions'));
    addpath(genpath('../tbxmanager'));

    disp 'COVERT ADVERSARY CONTROL';
    disp ''
    disp '-----------------------------------'
    disp ''
    

    TEST_EXAMPLE_inp = input('Use test example? <y/n>','s');
    if(TEST_EXAMPLE_inp == 'y' || TEST_EXAMPLE_inp == 'Y')
        TEST_EXAMPLE = true;
    else
        TEST_EXAMPLE = false;
    end
    disp '-----------------------------------'

    tic;

    AXISLIMIT = 8;
    ITERLIM = 100;

    %example data
    if(TEST_EXAMPLE == true)
        A = [0.9 0.2;0 0.9];
        B = [-0.3;-0.2];
        x0 = [2;0.5];
        G = [-1 -0.1;-0.5 0.9;0.9 0.4;-0.2 -1];
        w = [2.59;2.14;3.25;2.84];
        P = [-0.22 0.98;-0.66 -0.75;-0.96 0.29;0.91 -0.41;0.62 0.78];
        r = [0.53;0.71;0.34;2.66;2.03];
        umin = -1.9;
        umax = 3;
        v = [umax;-umin];
        IS_N_DIM = false;
        IS_3D = false;
    else
         disp 'Enter system matrix (A) '
            A = input('');
            disp 'Enter input matrix (B) '
            B = input('');
            disp 'Enter initial state (as a vector)'
            x0 = input('');
            disp 'Assuming alarm constraints of the form : Gx <= w'
            G = input('Enter G matrix : ');
            w = input('Enter w vector : ');
            disp 'Assuming desired operation area of the form : Px <= r';
            P = input('Enter P matrix : ');
            r = input('Enter r vector : ');
            disp 'Assuming linear input constr : umin<=u<=umax'
            umin = input('Enter umin : ');
            umax = input('Enter umax : ');
            AXISLIMIT = input('Enter limit of axes in plots : ');
            v = [umax;-umin];
            if(size(A,1) == 3)
                IS_3D = true;
            else
                IS_3D = false;
            end
            if(size(A,1) > 3)
                IS_N_DIM = true;
            else
                IS_N_DIM = false;
            end
    end
    
    disp 'Choose method for the target (equilibrium) point of the '
    disp 'expanding controller :'
    disp '1.Maximum distance from desired operation area'
    disp '2.Random choice '
    disp '3.Manual input from user '
    disp '----------------------------------------------------------'
    disp 'e.Show all valid equilibrium points of expanding controller'
    
    EQSTRAT = input('','s');
    
    if(EQSTRAT == 'e')
       showeqpoints(A,B,G,w,P,r,umin,umax,AXISLIMIT);
       return;
    end
    
    if(not(EQSTRAT == 'e'))
        disp '----------------------------------------------------------'
        disp 'Choose the strategy that the expanding controller will use '
        disp 'to reach its target point :'
        disp '1.Random facet'
        disp '2.Facet with best convergence rate'
        disp '3.Convex cone'
        fprintf('\n');

        ATTACKER_STRATEGY = input('','s');
        
        C_CHOSEN_COEFF = 0;
        if(ATTACKER_STRATEGY == '1' || ATTACKER_STRATEGY == '2')
            disp '----------------------------------------------------------'
            disp 'Note : With a smaller cdot value, the expanding controller will try force the state farther away from the desired operation ' 
            disp 'area, but with a cost of resources (it will control the state trajectory for a larger amount of time). ' 
            while(C_CHOSEN_COEFF <= 0 || C_CHOSEN_COEFF > 1)
               C_CHOSEN_COEFF = input('cdot/c = ');
               if(C_CHOSEN_COEFF <= 0 || C_CHOSEN_COEFF > 1)
                   disp 'This value must be higher than 0 and lower than 1.'
               end
            end
        end
        
    end
    
    %CONTRACTIVE ONLY
     %pre-processing
    [Kc1,e1] = kon_theorem(A,B,P,r,v); %desired operation area
    [Kc2,e2] = kon_theorem(A,B,G,w,v); %alarm constraints area

     %calculate trajectory
    [xcon,~] = covertContractive(A,B,G,w,Kc1,Kc2,x0,P,r,false,ITERLIM,1,0,0,0,0,0,0,0,0,0,0,0);

    %EXPANDING ONLY

     %pre-processing
    [ex,Kx,c,c_chosen,Q,q,q_x,L,l,z0,x0,xe,ue,CONVCONE] = covertExpandingInit(A,B,P,r,G,w,umin,umax,x0,C_CHOSEN_COEFF,ATTACKER_STRATEGY,EQSTRAT,AXISLIMIT);
    
     if(not(CONVCONE))
         %calculate trajectory
        [xexp,~] = covertExpandingTrajectory(A,B,G,w,umin,umax,x0,z0,xe,ue,ex,Kx,Q,q,q_x,L,l,c,c_chosen,false,ITERLIM,1);

         %define M and Mc and Mc'(named Mcnew in code)
         [x1_M,x2_M] = define_polyhedral(1,Q,q_x,0);

         q_Mc = c*q;
         q_x_Mc = q_Mc + Q*xe;
         [x1_Mc,x2_Mc] = define_polyhedral(1,Q,q_x_Mc,0);

         q_Mcnew = c_chosen*q;
         q_x_Mcnew = q_Mcnew + Q*xe;
         [x1_Mcnew,x2_Mcnew] = define_polyhedral(1,Q,q_x_Mcnew,0);

        %adversary control
        [xcon_adv,xexp_adv,nc,ne,con_exists] = covertAdversaryAlgorithm(A,B,G,w,P,r,Kc1,Kc2,Kx,umin,umax,c,z0,ex,Q,q,L,l,c_chosen,xe,ue,q_x,x0,ITERLIM);

        %DRAWING

        %contractive only (drawing)%%
        %desired operation area
        [x1_D,x2_D] = define_polyhedral(1,P,r,0);
        %alarm constraints area
        [x1_A,x2_A] = define_polyhedral(1,G,w,0);

        figure;
        plot(x1_D,x2_D,'g');
        hold on;
        plot(x1_A,x2_A,'r');

        plot(x0(1),x0(2),'k*');
        plot(xcon(1,:),xcon(2,:),'b.-');

        title('Contractive control only');
        legend('Desired Operation','Alarm constraints','Initial State','Trajectory');

        %draw axis lines
        axis square;
        xlim([-AXISLIMIT AXISLIMIT]);
        ylim([-AXISLIMIT AXISLIMIT]);
        xL = xlim;
        yL = ylim;
        line([0 0], yL);  %x-axis
        line(xL, [0 0]);  %y-axis
        xlabel('x1');
        ylabel('x2');

        %expanding only (drawing)%%

        %desired operation area
        [x1_D,x2_D] = define_polyhedral(1,P,r,0);
        %alarm constraints area
        [x1_A,x2_A] = define_polyhedral(1,G,w,0);

        figure;
        plot(x1_D,x2_D,'g');
        hold on;
        plot(x1_A,x2_A,'r');
        hold on;

        plot(x0(1),x0(2),'k*');
        hold on;
        plot(xexp(1,:),xexp(2,:),'m.-');
        hold on;
        plot(xe(1),xe(2),'c*');
        hold on;

        %
        plot(x1_M,x2_M,'k');
        hold on;
        plot(x1_Mc,x2_Mc,'Color',[0.7 0.7 0.7]);
        hold on;
        plot(x1_Mcnew,x2_Mcnew,'k--');
        %

        title('Expanding control only');
        legend('Desired Operation','Alarm constraints','Initial State','Trajectory','Expanding equilibrium','M','Mc','Mcdot');

        %draw axis lines
        axis square;
        xlim([-AXISLIMIT AXISLIMIT]);
        ylim([-AXISLIMIT AXISLIMIT]);
        xL = xlim;
        yL = ylim;
        line([0 0], yL);  %x-axis
        line(xL, [0 0]);  %y-axis
        xlabel('x1');
        ylabel('x2');

        % %adversary control (drawing)%%
        if(not(CONVCONE))
            %desired operation area
            [x1_D,x2_D] = define_polyhedral(1,P,r,0);
            %alarm constraints area
            [x1_A,x2_A] = define_polyhedral(1,G,w,0);

            %transform state vectors in form suitable for plotting
            counter = 1;
            if(con_exists)
                for i=1:size(xcon_adv,2)
                     if(not(and(abs(xcon_adv(1,i))<0.001,abs(xcon_adv(2,i))<0.001)))
                         xcon_adv_plot(:,counter) = xcon_adv(:,i);
                         counter = counter + 1;
                     end
                end
            end
            counter = 1;
            for i=1:size(xexp_adv,2)
                 if(not(and(abs(xexp_adv(1,i))<0.001,abs(xexp_adv(2,i))<0.001)))
                     xexp_adv_plot(:,counter) = xexp_adv(:,i);
                     counter = counter + 1;
                 end
            end
            %
            figure;
            plot(x1_D,x2_D,'g');
            hold on;
            plot(x1_A,x2_A,'r');
            hold on;
            plot(x0(1),x0(2),'ko');
            hold on;
            if(con_exists)
                plot(xcon_adv_plot(1,:),xcon_adv_plot(2,:),'b.-');
            end
            hold on;
            plot(xexp_adv_plot(1,:),xexp_adv_plot(2,:),'m.-');
            hold on;
            plot(xe(1),xe(2),'c*');

            %
            plot(x1_M,x2_M,'k');
            hold on;
            plot(x1_Mc,x2_Mc,'Color',[0.7 0.7 0.7]);
            hold on;
            plot(x1_Mcnew,x2_Mcnew,'k--');
            %


            title('Covert Adversary Control');
            if(con_exists)
                legend('Desired Operation','Alarm constraints','Initial State','Contractive','Expanding','Expanding equilibrium','M','Mc','Mcdot');
            else
                legend('Desired Operation','Alarm constraints','Initial State','Expanding','Expanding equilibrium','M','Mc','Mcdot');
            end


            %draw axis lines
            axis square;
            xlim([-AXISLIMIT AXISLIMIT]);
            ylim([-AXISLIMIT AXISLIMIT]);
            xL = xlim;
            yL = ylim;
            line([0 0], yL);  %x-axis
            line(xL, [0 0]);  %y-axis
            xlabel('x1');
            ylabel('x2');

        end

        %text output
        fprintf('\nSimulation terminated.');
        fprintf('\nContractive control active in %d',nc);
        fprintf(' time instants');
        fprintf('\nExpanding control active in %d',ne);
        fprintf(' time instants\n');

     end
     
     toc;
    
end