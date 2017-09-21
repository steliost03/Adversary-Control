%optimization problem : min{ex} so that: 
    
    % 1: S*(A+B*Kx) = H11*S
    % 2: L*(A+B*Kx) = H21*S+H22*L
    % 3: H22*l <= ex*l
    % 4: ex <= 1
    % 5: H11 >= O
    % 6: H21 >= O
    % 7: H22 >= O
    % 8: M>= O
    % 9: M*[S;L] = [Kx ; -Kx]
    % 10: M*[0;l] = [umax_new ; -umin_new]
    
    %(determine ex,Kx,H11,H21,H22,M)
    
    %H11 -> Hone
    %H21 -> Htwo
    %H22 -> Hthree
    
    %linprog setup
    %A,b: 3,4,5,6,7,8,10
    %Aeq,beq: 1,2,9
    
    %x(1) : ex
    %x(2,...,n+1):Kx
    %x(n+2,..,s*s+n+1) : H11
    
    %H11,H21,H22,M are represented by x with reversed columns.

function [Kout,e_out] = kon_theorem_modified(Asys,Bsys,Q,q,umax,umin,ue)

umax_new = umax - ue;
umin_new = umin - ue;

%find s: the number of q elements that are equal to zero
    qdim = size(Q,1);
    n = size(Asys,1);
    s = 0;
    for i=1:qdim
        if (abs(q(i)) < 0.001)
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
    
    n=2;
    
    Hone = sym('Hone',[s s]);
    Htwo = sym('Htwo',[qdim-s s]);
    Hthree = sym('Hthree',[qdim-s qdim-s]);
    M = sym('M',[2 qdim]);
    ex = sym('ex');
    Kx = sym('Kx',[1 n]);
    
    eq1_left = S*(Asys+Bsys*Kx);
    eq1_right = Hone*S;
    eq1 = eq1_left - eq1_right;
    
    eq2_left = L*(Asys+Bsys*Kx);
    eq2_right = Htwo*S+Hthree*L;
    eq2 = eq2_left - eq2_right;
    
    eq3_left = Hthree*l;
    eq3_right = ex*l;
    eq3 = eq3_left - eq3_right;
    
    eq4_left = ex;
    eq4_right = 1;
    eq4 = eq4_left - eq4_right;
    
    eq5_left = Hone;
    eq5_right = zeros(size(Hone));
    eq5 = eq5_left - eq5_right;
    
    eq6_left = Htwo;
    eq6_right = zeros(size(Htwo));
    eq6 = eq6_left - eq6_right;
    
    eq7_left = Hthree;
    eq7_right = zeros(size(Hthree));
    eq7 = eq7_left - eq7_right;
    
    eq8_left = M;
    eq8_right = zeros(size(M));
    eq8 = eq8_left - eq8_right;
    
    eq9_left = M*[S;L];
    eq9_right = [Kx ; -Kx];
    eq9 = eq9_left - eq9_right;
    
    eq10_left = M*[zeros(s,1);l];
    eq10_right = [umax_new ; -umin_new];
    eq10 = eq10_left - eq10_right;
    
    total_vars = 1+n+s*s+(qdim-s)*s+(qdim-s)*(qdim-s)+2*qdim;
    
    
    %1%
    A1 = zeros(size(eq1,2)*size(eq1,1),total_vars);
    b1 = zeros(size(eq1,2)*size(eq1,1),1);
    new_eq1_rowsize = size(eq1,1)*size(eq1,2);
    eq1 = reshape(eq1,[new_eq1_rowsize 1]);
    coefficients1 = zeros(size(eq1,1),s+2);
    
    for i=1:size(eq1,1)
        coefficients1(i,:) = coeffs(eq1(i));
    end
    
    %Kx coeffs determination
    for i=1:s
        A1(i,2) = coefficients1(i,2);
    end
    coeff_counter = 0;
    for i=s+1:2*s %can be generalized (n*s)
        coeff_counter = coeff_counter + 1;
        A1(i,3) = coefficients1(coeff_counter,2);
    end
    
    
    %Hone coeffs determination
    for i=1:s
        coeff_counter = 2;
        for j=2+n:1+n+s
            coeff_counter = coeff_counter+1;
            A1(i,j) = coefficients1(i,coeff_counter);
        end
    end
    coeff_counter1 = s;
    for i=s+1:2*s %can be generalized (n*s)
        coeff_counter2 = 2;
        coeff_counter1 = coeff_counter1 + 1;
        for j=2+n:1+n+s
            coeff_counter2 = coeff_counter2+1;
            A1(i,j) = coefficients1(coeff_counter1,coeff_counter2);
        end
    end
    
    %
    for i=1:size(eq1,1)
        b1(i) = -coefficients1(i,1);
    end
    
    
    %2%
    new_eq2_rowsize = size(eq2,1)*size(eq2,2);
    eq2 = reshape(eq2,[new_eq2_rowsize 1]);
    A2 = zeros(size(eq2,1),total_vars);
    b2 = zeros(size(eq2,1),1);
    coefficients2 = zeros(size(eq2,1),qdim-s+s+2);
    
    for i=1:size(eq2,1)
        coefficients2(i,:) = coeffs(eq2(i));
    end
    
    %Kx coeffs determination
    for i=1:qdim-s
        A2(i,2) = coefficients2(i,2);
    end
    
    for i=qdim-s+1:(qdim-s)*n
        A2(i,n+1) = coefficients2(i,2);
    end
    
    
    %Htwo coeffs determination
    j_begin = n + 2+s*s;
    j_end = n+s+1+s*s;
    for i=1:qdim-s
        coeff_counter = n+1;
        for j=j_begin:j_end
            A2(i,j) = coefficients2(i,coeff_counter);
            coeff_counter = coeff_counter + 1;
        end
        j_begin = n+s+2+s*s;
        j_end = j_begin + s-1;
    end
    
    j_begin = n + 2+s*s;
    j_end = n+s+1+s*s;
    for i=qdim-s+1:(qdim-s)*n
        coeff_counter = n+1;
        for j=j_begin:j_end
            A2(i,j) = coefficients2(i,coeff_counter);
            coeff_counter = coeff_counter + 1;
        end
        j_begin = n+s+2+s*s;
        j_end = j_begin + s-1;
    end
    
    %Hthree coeffs determination
    j_begin = n+1+s*s+(qdim-s)*s+1;
    temp = j_begin;
    j_end = j_begin + qdim - s - 1;
    for i=1:qdim-s
        coeff_counter = s+3;
        for j=j_begin:j_end
            A2(i,j) = coefficients2(i,coeff_counter);
            coeff_counter = coeff_counter + 1;
        end
        j_begin = j_end+1;
        j_end = j_begin + qdim-s - 1;
    end
    
    j_begin = temp;
    j_end = j_begin + qdim - s - 1;
    for i=qdim-s+1:(qdim-s)*n
        coeff_counter = s+3;
        for j=j_begin:j_end
            A2(i,j) = coefficients2(i,coeff_counter);
            coeff_counter = coeff_counter + 1;
        end
        j_begin = j_end+1;
        j_end = j_begin + qdim-s-1;
    end
    
    %
    for i=1:size(eq2,1)
        b2(i) = -coefficients2(i,1);
    end
    
    %3%
    A3 = zeros(size(eq3,1),total_vars);
    b3 = zeros(size(eq3,1),1);
    coefficients3 = zeros(size(eq3,1),qdim-s+1);
    
    for i=1:size(eq3,1)
        coefficients3(i,:) = coeffs(eq3(i));
    end
    
    %ex coeffs determination
    for i=1:size(eq3,1)
        A3(i,1) = coefficients3(i,1);
    end
    
    %Hthree coeffs determination
    j = n + 2 +s*s + (qdim-s)*s;
    for i=1:size(eq3,1)
        for w=2:qdim-s+1
            A3(i,j) = coefficients3(i,w);
            j = j + 1;
        end
    end
    %4%
    A4 = zeros(1,total_vars);
    A4(1,1) = 1;
    b4 = 1;
   
    %5%
    A5 = zeros(s*s,total_vars);
    b5 = zeros(s*s,1);
    
    j = n+2;
    for i=1:s*s
        A5(i,j) = -1;
        j = j + 1;
    end
    
    %6%
    A6 = zeros((qdim-s)*s,total_vars);
    b6 = zeros((qdim-s)*s,1);
    
    j = s*s+n+2;
    for i=1:(qdim-s)*s
        A6(i,j) = -1;
        j = j + 1;
    end
    
    %7%
    A7 = zeros((qdim-s)*(qdim-s),total_vars);
    b7 = zeros((qdim-s)*(qdim-s),1);
    
    j = s*s+n+2+(qdim-s)*s;
    for i=1:(qdim-s)*(qdim-s)
        A7(i,j) = -1;
        j = j + 1;
    end
    
    %8%
    A8 = zeros(2*qdim,total_vars);
    b8 = zeros(2*qdim,1);
    
    j = s*s+n+2+(qdim-s)*s+(qdim-s)*(qdim-s);
    for i=1:2*qdim
        A8(i,j) = -1;
        j = j + 1;
    end
    
    %9%
    new_eq9_rowsize = size(eq9,1)*size(eq9,2);
    eq9 = reshape(eq9,[new_eq9_rowsize 1]);
    A9 = zeros(size(eq9,1),total_vars);
    b9 = zeros(size(eq9,1),1);
    coefficients9 = zeros(size(eq9,1),qdim+1);
    
    for i=1:size(eq9,1)
        coefficients9(i,:) = coeffs(eq9(i)); 
    end
    
    %Kx coeffs determination
    A_counter = 0;
    for j=1:n
        for i=1:2
            A_counter = A_counter + 1;
            A9(A_counter,1+j) = coefficients9(i,qdim+1);
        end
    end
   
    
    %M coeffs determination
    
    j_begin = total_vars - 2*qdim+1;
    j = j_begin;
    for i=1:2 %can be generalized
        for w=1:qdim
            A9(i,j) = coefficients9(i,w);
            j = j + 1;
        end
    end
    
    
    j = j_begin;
    for i=3:4 %can be generalized
        for w=1:qdim
            A9(i,j) = coefficients9(i,w);
            j = j + 1;
        end
    end
    
     
    %10%
    A10 = zeros(size(eq10,1),total_vars);
    coefficients10 = zeros(size(eq10,1),qdim-s+1);
    b10 = [umax_new ; -umin_new];
    
    for i=1:size(eq10,1)
        coefficients10(i,:) = coeffs(eq10(i));
    end
    
    %qdim-s values of M will not be zero (s values will be zero)
    col_mark = n+1+s*s+(qdim-s)*s+(qdim-s)*(qdim-s);
    j_begin = 1+col_mark;
    for i=1:size(eq10,1)
        coeff_counter = 2;
        for j=j_begin:qdim+col_mark-s
            A10(i,j) = coefficients10(i,coeff_counter);
            coeff_counter = coeff_counter + 1;
        end
       col_mark = total_vars - qdim;
       j_begin = col_mark+1;
    end
    
    
    A = [A3;A4;A5;A6;A7;A8;A10];
    b = [b3;b4;b5;b6;b7;b8;b10];
    Aeq = [A1;A2;A9];
    beq = [b1;b2;b9];
    f = zeros(1,total_vars);
    f(1) = 1;
    
    options = optimset('Display', 'off') ;
    x = linprog(f,A,b,Aeq,beq,[],[],[],options);
    
    e_out = x(1);
    Kout = zeros(1,n);
    for i=1:n
        Kout(i) = x(i+1);
    end       
end