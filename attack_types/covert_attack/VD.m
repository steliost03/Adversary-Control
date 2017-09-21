function f = VD(x,xsym,P,r)

    q = size(P,1);
     
    Px = P*xsym;
    Px = vpa(Px);
    
    Px_coeffs = zeros(q,2);
    
    for i=1:q
        Px_coeffs(i,:) = coeffs(Px(i));
    end
    for i=1:q
        temp = Px_coeffs(i,2);
        Px_coeffs(i,2) = Px_coeffs(i,1);
        Px_coeffs(i,1) = temp;
    end
    
    for i=1:q
        Px_l_x1(i) = Px_coeffs(i,1) / r(i);
        Px_l_x2(i) = Px_coeffs(i,2) / r(i);
    end
    
    Px_l = [Px_l_x1' Px_l_x2'];
    
    f = vpa(Px_l*xsym,3);
    f = subs(f,'x1',x(1));
    f = subs(f,'x2',x(2));
    f = double(f);
    f = max(f);
    f = -f;
 
    
end