function result = TrajectoryTermination(dim,xcurrent,xe)
    
    result = 1;
    for i=1:dim
        if( abs(xcurrent(i) - xe(i)) > 0.1)
            result = 0;
            return;
        end 
    end
end