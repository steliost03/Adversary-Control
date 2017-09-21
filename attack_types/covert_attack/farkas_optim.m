%solves the optimization problem :
%determine c,E such that max{c} while respecting the conditions :
% EQ = W (1) , E(cq) <= r (2) , E >= O (3) , c < 1 (4) , -c < 0 (5)
%where : Mc = { z : Qz <= cq } and Hv : { z : Wz <= w}
%(Farka's Lemma : implies Mc(Q,cq) is a subset of Hv(W,w))
% E dimensions : 1Xqdim ,where qdim the number of rows of Q.
% Q and W column number must be 2 (special case for 2x2 systems)

function c = farkas_optim(Q,q,W,w,posOfZero)
  
   qdim = size(Q,1);
    
    %set up fmincon
    %A,b : 3,4
    %Aeq,beq : 1
    %nonlcon : 2
    
    %x(1) : c
    %x(2,...,qdim+1): E (with reversed columns)
     
    total_vars = qdim + 1;
    
    E = sym('E',[1 qdim]);
    c = sym('c');
    O = zeros(1,qdim);
    
    eq1_left = E*Q;
    eq1_right = W;
    eq1 = eq1_left - eq1_right;
    
    eq2_right = w;
    eq2_left = E*(c*q);
    eq2 = eq2_left - eq2_right;
    
    eq3_left = E;
    eq3_right = O;
    eq3 = eq3_left - eq3_right;
    
    eq4_left = 1;
    eq4_right = c;
    eq4 = eq4_left- eq4_right;
    
    eq5_left = 0 ;
    eq5_right = -1;
    eq5 =  eq5_left - eq5_right;
    
    
    
    %1
    coefficients1 = zeros(size(eq1,2),qdim+1);
    A1 = zeros(size(eq1,2),qdim+1);
    
    for i=1:size(eq1,2)
        coefficients1(i,:) = coeffs(eq1(i));
    end
    
    for i=1:size(eq1,2)
        for j=2:size(coefficients1,2)
            A1(i,j) = coefficients1(i,j);
        end
    end
   
    b1 = coefficients1(:,1);
    b1 = -b1;
   
    
    
    %3
    b3 = zeros(total_vars-1,1);
    A3 = zeros(total_vars-1,total_vars);
    
    A3rowcounter = 1;
    for i=2:total_vars
        A3(A3rowcounter,i) = -1;
        A3rowcounter = A3rowcounter + 1;
    end
    
    %4
    
    b4 = zeros(2,1);
    b4(1) = 1;
    b4(2) = 0;

    A4 = zeros(2,total_vars);
    A4(1,1) = 1;
    A4(2,1) = -1;
  
   
    
    A = A3;
    b = b3;
    

    Aeq = A1;
    beq = b1;
    
    x0 = zeros(total_vars,1);
    fun = @(x)(-x(1)); 
    
    lb(1) = 0;
    ub(1) = 1;
    for i = 2:total_vars
        lb(i) = -0.01;
        ub(i) = 5;
    end
    
    f = zeros(size(x0));
    options = optimoptions('linprog','Display','off');
    %first find a feasible point to serve as initial point for the
    %optimization
    xinitial = linprog(f,[],[],Aeq,beq,lb,ub,[],options);
    
    x0 = xinitial;
    x0(1) = 0.01;
    options = optimoptions('fmincon','Display','off','TolCon',1e-15,'TolFun',1e-15,'TolX',1e-15);
    [x,fval] = fmincon(fun,x0,[],[],Aeq,beq,lb,ub,@(x)nonlcon_farkoptim(x,eq2,qdim,total_vars,posOfZero),options); 
    
    c = x(1);
end