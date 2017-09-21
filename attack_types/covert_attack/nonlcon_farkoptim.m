%representation of nonlinear constraints for fmincon, in farkas_optim.m
function [c,ceq] = nonlcon_farkoptim(x,equation,q,varNum,posOfZero)

    ceq = [];

    equation = vpa(equation); %evaluate equation (originally in symbolic representation)
    
    Enames = cell(1,q);
    
    for i=1:q
        i_str = sprintf('%d',i); %convert to string
        E_str = strcat('E',i_str); %concatenate strings
        Enames{1,i} = E_str;
    end
    
    EnamesCounter = 0;
    %x(1) is reserved for c
    %E columns are reversed.
    for i=varNum:-1:2
        EnamesCounter = EnamesCounter + 1;
        if(EnamesCounter == posOfZero)
            continue
        end
        equation = subs(equation,Enames{1,EnamesCounter},x(i));
    end
    
    equation = subs(equation,'c',x(1));
    equation = vpa(equation);
    equation = double(equation);
    %c = equation - 0.5;
    c = equation;
    
end