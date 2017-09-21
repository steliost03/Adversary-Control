%P,r -> desired operation area ( Px <= r)
%
function u = contractive_algorithm(A,B,P,r,v,x_current,umax,umin,Kinput)

     Kinitial = Kinput;
     
     if(is_inside_conv_area(P,r,x_current))
         u = Kinitial*x_current;
     else
         disp('*');
         disp('State trajectory outside desired operation area:');
         disp('Contractive controller will adjust its control law');
         disp('*');
         %c = lyapunov_value(P,r,x_current);
         c = lyapunov_value(2,P,r,x_current);
         rnew = c*r;
         [K,~,Kexists] = kon_theoremFeasibilityIncluded(A,B,P,rnew,v);
         if(Kexists)
             u = K*x_current;
         else
             if(Kinitial*x_current > umax)
                 u = umax;
             else
                 u = umin;
             end
         end
     end  
end






