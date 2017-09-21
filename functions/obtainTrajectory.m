function [x,count] = obtainTrajectory(A,B,K,x0,xe,ue,iterlim)
    
    dim = size(x0,1);
    x(:,1) = x0;
    u(1) = ue + K*(x0-xe);
    
    count = 1;
    terminationCondition = TrajectoryTermination(dim,x0,xe);
    while( and(count < iterlim,not(terminationCondition)))
      
        x(:,count+1) = A*x(:,count) + B*(K*x(:,count)) - A*xe - B*(K*xe) + xe;
        u(count+1) = ue + K*(x(:,count+1) - xe);
        terminationCondition = TrajectoryTermination(dim,x(:,count),xe);
        count = count + 1;
    end
    
    count = count - 1;

end