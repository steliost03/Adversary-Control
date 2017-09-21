function [c,ceq] = nonlcon_prop(x,equation,m,n)

    c = [];
    
    %rename symbolic variables to x(..)
    
    Knames = cell(1,n);
    for i=1:n
        s_no = sprintf('%d',i);
        Knames{1,i} = strcat('K',s_no);
    end
    Hnames = cell(1,m*m);
    Hcounter = 0;
    for i=1:m
        for j=1:m
            i_str = sprintf('%d',i);
            H_str_begin = strcat('H',i_str);
            H_str_begin = strcat(H_str_begin,'_');
            Hcounter = Hcounter + 1;
            j_str = sprintf('%d',j);
            Hnames{1,Hcounter} = strcat(H_str_begin,j_str);
        end
    end

    xnames = cell(1,m*m+n);

    for i=1:(m*m+n)
        i_str = sprintf('%d',(i+1));
        x_str = strcat('x(',i_str);
        x_str = strcat(x_str,')');
        xnames{1,i} = x_str;
    end

    for i=1:m*m
        equation = subs(equation,Hnames{1,i},xnames{1,i});
    end
    
    
    j = 0;
    for i=(m*m+1):(m*m+n)
        j = j+1;
        equation = subs(equation,Knames{1,j},xnames{1,i});
    end
    
    equation = vpa(equation,2);
    for i=1:m*m+n
        equation = subs(equation,xnames{1,i},x(i+1));
    end
    
    equation = double(equation);
    ceq = equation;
        
end