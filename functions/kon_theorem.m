%equations and inequalities :

%1: P(A+BK) = HP
%2: Hr <= er
%3: H >= O
%4: e <= 1
%5: MP = F
%6: Mr <= v
%7: M >= O

%this function determines (via optimization) e,K,H,M so that e is
%minimized (while satisfying the above).

%linprog setup:
%A,b : 2,3,4,6,7
%Aeq,beq : 1,5

%x(1) : e
%x(2),...,x(m*m+1) : H
%x(m*m+1),...,x((m+2)*m+1) : M
%x( (m+2)*m+1 ),...,x( (m+2)*m+1+n ) : K

%For 2x2 systems only (n=2)

function [Kout,e_out] = kon_theorem(Asys,B,P,r,v)
    
    m = size(P,1);
    n = length(Asys);
    
    H = sym('H',[m m]);
    O_H = zeros(m,m);
    K = sym('K',[1 n]);
    e = sym('e');
    F = [K ; -K];
    M = sym('M',[2 m]);
    O_M = zeros(2,m);
    
    total_vars = (m+2)*m+1+n;
    
    eq1_left = P*(Asys+B*K);
    eq1_right = H*P;
    eq1 = eq1_left - eq1_right;
    
    eq2_left = H*r;
    eq2_right =  e*r;
    eq2 = eq2_left - eq2_right;
    
    eq3_left = -H;
    eq3_right = O_H;
    eq3 = eq3_left - eq3_right;
    
    eq4_left = e;
    eq4_right = 1;
    eq4 = eq4_left - eq4_right;
    
    eq5_left = M*P;
    eq5_right = F;
    eq5 = eq5_left - eq5_right;
    
    eq6_left = M*r;
    eq6_right = v;
    eq6 = eq6_left - eq6_right;
    
    eq7_left = -M;
    eq7_right = O_M;
    eq7 = eq7_left - eq7_right;
    
     
    %1%
    new_eq1_rowsize = size(eq1,1)*size(eq1,2);
    eq1 = reshape(eq1,[new_eq1_rowsize,1]);
    
    A1 = zeros(size(eq1,1),total_vars);
    b1 = zeros(size(eq1,1),1);
    
    coefficients1 = zeros(size(eq1,1),m+2);
    
    for i=1:size(eq1,1)
        coefficients1(i,:) = coeffs(eq1(i));
    end
    
    for i=1:size(eq1,1)
        b1(i) = -coefficients1(i,1);
    end
    
    %can be generalized
    %K coefficients
    for i=1:m
        A1(i,(m+2)*m+2) = coefficients1(i,2);
    end
    for i=m+1:2*m
        A1(i,(m+2)*m+3) = coefficients1(i,2);
    end
    
    %H coefficients
    col_counter = 0;
    for i=1:m
        col_counter = col_counter + 1;
        z = col_counter*m+1;
        for j=3:m+2
            A1(i,z) = coefficients1(i,j);
            z = z - 1;
        end
    end
    
    col_counter = 0;
    for i=m+1:2*m
        col_counter = col_counter + 1;
        z = col_counter*m+1;
        for j=3:m+2
            A1(i,z) = coefficients1(i,j);
            z = z - 1;
        end
    end
    
    
    %2%
    coefficients2 = zeros(size(eq2,1),m+1);
    
    b2 = zeros(size(eq2,1),1);
    A2 = zeros(size(eq2,1),total_vars);
    
    for i=1:size(eq2,1)
        coefficients2(i,:) = coeffs(eq2(i));
    end
    
    %e
    for i=1:size(eq2,1)
        A2(i,1) = coefficients2(i,1);
    end
    
    %H coefficients
    col_counter = 0;
    for i=1:m
        col_counter = col_counter + 1;
        z = col_counter * m + 1;
        for j=2:m+1
            A2(i,z) = coefficients2(i,j);
            z = z - 1;
        end
    end
    
    %3%
    b3 = zeros(m*m,1);
    A3 = zeros(m*m,total_vars);
    
    j = 2;
    for i=1:m*m
        A3(i,j) = -1;
        j = j + 1;
    end
    
    %4%
    A4 = zeros(1,total_vars);
    b4 = 1;
    A4(1,1) = 1;
    
    %5%
    new_eq5_rowsize = size(eq5,1)*size(eq5,2);
    eq5 = reshape(eq5,[new_eq5_rowsize 1]);
    A5 = zeros(2*m,total_vars);
    b5 = zeros(2*m,1);
    coefficients5 = zeros(size(eq5,1),m+1);

    for i=1:size(eq5,1)
        coefficients5(i,:) = coeffs(eq5(i));
    end
    
    %can be generalized
    %M coefficients
    col_counter = 0;
    for i=1:2
        col_counter = col_counter + 1;
        z = m*m+1+col_counter*m;
        for j=1:m
            A5(i,z) = coefficients5(i,j);
            z = z-1;
        end
    end
    
    col_counter = 0;
    for i=3:4
        col_counter = col_counter + 1;
        z = m*m+1+col_counter*m;
        for j=1:m
            A5(i,z) = coefficients5(i,j);
            z = z-1;
        end
    end
    
    %can be generalized
    %K coefficients
    for i=1:2
        A5(i,(m+2)*m+2) = coefficients5(i,m+1);
    end
    
    for i=3:4
        A5(i,(m+2)*m+3) = coefficients5(i,m+1);
    end
        
    
    %6%
    A6 = zeros(size(eq6,1),total_vars);
    b6 = zeros(size(eq6,1),1);
    coefficients6 = zeros(size(eq6,1),m+1);
    
    for i=1:size(eq6,1)
        coefficients6(i,:) = coeffs(eq6(i));
    end
    
    %7%
    b7 = zeros(2*m,1);
    A7 = zeros(2*m,total_vars);
    
    j = m*m+2;
    for i=1:2*m
        A7(i,j) = -1;
        j = j + 1;
    end
   
    A = [A2;A3;A4;A6;A7];
    Aeq = [A1;A5];
    b = [b2;b3;b4;b6;b7];
    beq = [b1;b5]; 
    
    f = zeros(1,total_vars);
    f(1) = 1;
    
    options = optimset('Display', 'off') ;
    x = linprog(f,A,b,Aeq,beq,[],[],[],options);
    
    %K determination
    Kout = zeros(1,n);
    K_counter = 0;
    for i=(m+2)*m+2:(m+2)*m+1+n
        K_counter = K_counter + 1;
        Kout(K_counter) = x(i);
    end
    %e determination
    e_out = x(1);
     
end