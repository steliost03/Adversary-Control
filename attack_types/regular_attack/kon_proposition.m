%1 : [G;F](A+BK) = H[G;F]
%2 : H[w;v] <= e[w;v]
%3 : e<=1
%4 : H>=O

%fmincon setup:
%A,b : 2,3,4
%ceq(x) : 1
%all other parameters : []

function [Kout,e_out] = kon_proposition(Asys,B,G,w,v)

m = length(w);
m = m+2;
n = length(Asys);
    
H = sym('H',[m m]);
O = zeros(m,m);
K = sym('K',[1 n]);
e = sym('e');
F = [K ; -K];

P = [G;F];
r = [w;v];

eq1_left = P*(Asys+B*K);
eq1_right = H*P;

eq1 = eq1_left - eq1_right;
    
eq2_left = H*r;
eq2_right = e*r;
 
eq2 = eq2_left - eq2_right;
    
eq3_left = e;
eq3_right = 1;
 
eq3 = eq3_left - eq3_right;
    
eq4_left = H;
eq4_right = O;

eq4 = eq4_left - eq4_right;


%set up fmincon
fun = @(x)x(1);
Aeq = [];
beq = [];
lb = [];
ub = [];

total_vars = m*m+n+1;
x0 = zeros(1,total_vars);
x0(1) = 1;

%eq2
coefficients = zeros(length(eq2),m+1);
for i=1:length(eq2)
    coefficients(i,:) = coeffs(eq2(i));
end

if(mod(size(coefficients,2)-1,2)==0)
    lim = (size(coefficients,2)-1)/2;
else
    lim = ((size(coefficients,2))/2)-1;
end

%swap all columns
for w=1:size(coefficients,1)
   j = size(coefficients,2);
   for i=2:lim+1
        temp = coefficients(w,i);
        coefficients(w,i) = coefficients(w,j);
        coefficients(w,j) = temp;
        j = j - 1;
   end
end

Aeq2 = zeros(size(coefficients,1),m*m+1);
beq2 = zeros(size(coefficients,1),1);

col_counter = 1;
row_mark = 2;
while(row_mark<m*m)
    j = 2;
    for i=row_mark:row_mark+m-1
        Aeq2(col_counter,i) = coefficients(col_counter,j);
        j = j + 1;
    end
    col_counter = col_counter + 1;
    row_mark = row_mark+m;
end

for i=1:size(coefficients,1)
    Aeq2(i,1) = coefficients(i,1);
end

%eq3
Aeq3 = zeros(1,m*m+1);
beq3 = 1;
Aeq3(1) = 1;

%eq4
Aeq4 = zeros(m*m,m*m+1);
beq4 = zeros(m*m,1);
j = 2;
for i=1:m*m
    Aeq4(i,j) = -1;
    j = j + 1;
end

A = [Aeq2;Aeq3;Aeq4];
b = [beq2;beq3;beq4];
zero_cols = zeros(size(A,1),n);
A = [A zero_cols];

%eq1
newrowsize = size(eq1,1)*size(eq1,2);
eq1 = reshape(eq1,[newrowsize,1]);

%
parpool('local');
options = optimoptions('fmincon','TolCon',1e-4,'TolFun',1e-4,'TolX',1e-8,'MaxFunEvals',9000,'MaxIter',3000,'UseParallel',true);
tic
x = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,@(x)nonlcon_prop(x,eq1,m,n),options);
toc
%

Kout = zeros(1,n);
K_counter = 0;
for i=m*m+2:m*m+1+n
    K_counter = K_counter + 1;
    Kout(K_counter) = x(i);
end
e_out = x(1);

delete(gcp);

end