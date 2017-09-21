%Defines 2D polyhedral from equation :
%Gx <= w if type=1
%|Gx| <=w if type=2
% w<= Gx <= wup if type=3
function [polyhedral_x1,polyhedral_x2] = define_polyhedral(type,G,w,wup)
    
    
    %step 1: define the lines
    if(type == 1)
        lineCount = size(G,1);
        constraint_lines = zeros(2,2,lineCount);
        for i=1:lineCount
             constraint_lines(:,:,i) = [1000 -1000;(w(i)-(1000*G(i,1)))/G(i,2) (w(i)+(1000*G(i,1)))/G(i,2)];
        end
    end
    
    if(type == 2)
        lineCount = 2*size(G,1);
        constraint_lines = zeros(2,2,lineCount);
        line_counter = 0;
        for i=1:size(G,1)
            line_counter = line_counter + 1;
            constraint_lines(:,:,line_counter) = [1000 -1000;(w(i)-(1000*G(i,1)))/G(i,2) (w(i)+(1000*G(i,1)))/G(i,2)];
            line_counter = line_counter + 1;
            constraint_lines(:,:,line_counter) = [1000 -1000;(-w(i)-(1000*G(i,1)))/G(i,2) (-w(i)+(1000*G(i,1)))/G(i,2)];
        end
        
        Gnew = zeros(2*size(G,1),size(G,2));
        wnew = zeros(2*size(w,1),1);  
        Gnew_counter = 1;
        for i=1:size(G,1)
            
            Gnew(Gnew_counter,:) = G(i,:);
            wnew(Gnew_counter) = w(i);
            Gnew_counter = Gnew_counter + 1;
            Gnew(Gnew_counter,:) = G(i,:);
            wnew(Gnew_counter) = w(i);
        end
        
        G = Gnew;
        w = wnew;
    end
    
    if(type == 3)
        Gnegative = -G;
        lineCount = 2*size(G,1);
        line_counter = 0;
        for i =1:size(G,1)
            line_counter = line_counter + 1;
            constraint_lines(:,:,line_counter) = [1000 -1000;(wup(i)-(1000*G(i,1)))/G(i,2) (wup(i)+(1000*G(i,1)))/G(i,2)];
            line_counter = line_counter + 1;
            constraint_lines(:,:,line_counter) = [1000 -1000;(-w(i)-(1000*Gnegative(i,1)))/Gnegative(i,2) (-w(i)+(1000*Gnegative(i,1)))/Gnegative(i,2)];
        end
        
        Gnew = zeros(2*size(G,1),size(G,2));
        wnew = zeros(2*size(w,1),1);  
        Gnew_counter = 1;
        for i=1:size(G,1)
            
            Gnew(Gnew_counter,:) = G(i,:);
            wnew(Gnew_counter) = w(i);
            Gnew_counter = Gnew_counter + 1;
            Gnew(Gnew_counter,:) = G(i,:);
            wnew(Gnew_counter) = w(i);
        end
        
        G = Gnew;
        w = wnew;
    end

    %step 2: define the half planes
    line_planes = zeros(2,4,lineCount);
    for i=1:lineCount
        testpoint = point_perp(constraint_lines(:,:,i),constraint_lines(:,1,i),1000);
        testpoint_x1 = testpoint(1,:);
        testpoint_x2 = testpoint(2,:);
        %test if direction of half plane is correct
        eq_value1 = G(i,:)*testpoint;
        if(eq_value1 > w(i))
            farawaypoint1 = point_perp(constraint_lines(:,:,i),constraint_lines(:,1,i),-1000);
            point_perp_value = -1000;
        else
            farawaypoint1 = [testpoint_x1;testpoint_x2];
            point_perp_value = 1000;
        end
        farawaypoint2 = point_perp(constraint_lines(:,:,i),constraint_lines(:,2,i),point_perp_value);
        line_planes(:,:,i) = [constraint_lines(:,1,i),constraint_lines(:,2,i),farawaypoint1,farawaypoint2];
        
    end
    
    %find the intersection of all the planes
    [polyhedral_x1,polyhedral_x2] = polybool('intersection',line_planes(1,:,1),line_planes(2,:,1),line_planes(1,:,2),line_planes(2,:,2));
    for i=3:lineCount
        [polyhedral_x1,polyhedral_x2] = polybool('intersection',line_planes(1,:,i),line_planes(2,:,i),polyhedral_x1,polyhedral_x2);
    end
    
 
end