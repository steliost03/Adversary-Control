function [e,K,c,c_chosen,Q,q,q_x,L,l,z0,x0,xe,ue,convconestrat] = covertExpandingInit(A,B,P,r,G,w,umin,umax,x0,c_chosen_coeff,attack_strat,eqstrat,axislimit)
    
    clk = clock + 10*5;
    rng(round(10000*clk(6)));
    
    %(xe,ue) determination
    [xe,ue] = expanding_eqpoint(A,B,P,r,G,w,umin,umax,eqstrat);
    z0 = x0 - xe;
    umax_new = umax - ue;
    umin_new = umin - ue;
    vnew = [umax_new ; -umin_new];
    
    %initialization for strategies 1 and 2
    if(attack_strat == '1' || attack_strat == '2')

        %Kx determination

        %M_z = {z : Qz << q}
        %Q = [G  -P_v_inv]
        %q = [wnew  c0*r_v_new]

        %Determine all H_j for j=1,...,q
        %H_j = { x : -Pj_inv <= -r_j}

        %count and record all possible suitable facets.
        q = size(P,1); %q : total no. of facets of desired area
        v_index = 0; %the index of the facet we will finally choose.
        v_indices = cell(1,1); %we store the indices of all the candidate
        %facets of the desired operation in this cell array.

        vindic_count = 0;
        for j=1:q
            Pj_inv = P(j,:); 
            rj = r(j);
            if(is_inside_conv_area(-Pj_inv,-rj,xe))
                v_index = j;
                vindic_count = vindic_count + 1;
                v_indices{1,vindic_count} = v_index;
            end
        end
    
    end
  
    %choice of facet is dependent on the chosen strategy:
    if(attack_strat == '1')
        %strategy 1 : choose random facet from available ones.
        convconestrat = false;
        s = size(v_indices,2);
        random_index = round(s*rand);
        if(random_index == 0)
            random_index = 1;
        end
        v_index = v_indices{1,random_index};
        fprintf('\n%d facet(s) suitable for the expanding controller.',s);
        fprintf(' Chosen facet index : %d\n',v_index);
        [e,K,Q,q] = covert_calcK(v_index,A,B,x0,xe,P,r,G,w,umin,umax,ue);
        P_v_inv = P(v_index,:);
        r_v_new = r(v_index);
    elseif(attack_strat == '2')
        %strategy 2 : Choose facet with best convergence rate (smallest e)
        convconestrat = false;
        e_values = zeros(1,size(P,1));
        for i=1:size(P,1)
            e_values(i) = 9999;
        end
        K_values = cell(1,size(P,1));
        Q_values = cell(1,size(P,1));
        q_values = cell(1,size(P,1));
        for count = 1:size(P,1)
           %first determine if facet can be used
           Pj_inv = P(count,:); 
           rj = r(count);
           if(not(is_inside_conv_area(-Pj_inv,-rj,xe)))
                continue;
           end
           %if it can,calculate convergence rate
          [e_curr,K_curr,Q_curr,q_curr] = covert_calcK(count,A,B,x0,xe,P,r,G,w,umin,umax,ue);
          e_values(count) = e_curr;
          K_values{1,count} = K_curr;
          Q_values{1,count} = Q_curr;
          q_values{1,count} = q_curr;
        end
        [e,minindex] = min(e_values);
        K = K_values{1,minindex};
        Q = Q_values{1,minindex};
        q = q_values{1,minindex};
        P_v_inv = P(minindex,:);
        r_v_new = r(minindex);
        fprintf('\nThe chosen facet produces the following values :')
        fprintf('\n e : %f',e);
        fprintf('\n K :\n');
        for i=1:length(K)
            fprintf('%f   ',K(i));
        end
        fprintf('\n');
        
    elseif(attack_strat == '3')
        %strategy 3 : Form a convex cone from the suitable facet.
        convconestrat = true;
        convcone(A,B,x0,G,w,P,r,umin,umax,xe,ue,axislimit);
        e = 0;
        K = 0;
        c = 0;
        c_chosen = 0;
        Q = 0;
        q = 0;
        q_x = 0;
        L = 0;
        l = 0;
        z0 = 0;
        x0 = 0;
        xe = 0;
        ue = 0;
        return
    end

   
   %pre-processing for switching signal 
   posOfZero = 0;
   for i=1:length(q)
       if(abs(q(i)) < 0.001)
           q(i) = 0;
           posOfZero = i;
       end
   end
    
    r_v_new_z = -r_v_new + P_v_inv*xe;
    c = farkas_optim(Q,q,-P_v_inv,r_v_new_z,posOfZero);
    %c = farkasoptim_mpt(Q,q,-P_v_inv,r_v_new);
    c_chosen = c_chosen_coeff*c;
   
    %first switching signal calc
    qdim = size(Q,1);
    n = size(A,1);
    
     %find s: the number of q elements that are equal to zero
    s = 0;
    for i=1:qdim
        if (q(i) == 0)
            s = s+1;
        end
    end
    
    %define matrices S and L 
    S = zeros(s,n);
    for i=1:s
        S(i,:) = Q(i,:);
    end
    L = zeros(qdim-s,n);
    l = zeros(qdim-s,1);
    Lcounter = 0;
    for i=s+1:qdim
        Lcounter = Lcounter + 1;
        L(Lcounter,:) = Q(i,:);
        l(Lcounter) = q(i);
    end
    
    q_x = q+Q*xe;
    
end