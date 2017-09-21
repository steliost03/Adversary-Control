%returns true if given point is on boundary of given polyhedral
%polyhedral : { x : Px <= r }
function result = onBoundary(point,P,r)
    
    facetCount = size(P,1);
    eq_satisfied = false;
    other_ineqs_satisfied = true;
    for i=1:facetCount
        if( abs((P(i,:)*point(:)) - r(i)) < 0.05)
            eq_satisfied = true;
            break;
        end
    end
    for i=1:facetCount
        if ( (P(i,:)*point(:) - r(i)) > 0.05)
            other_ineqs_satisfied = false;
            break;
        end
    end
    
    result = eq_satisfied && other_ineqs_satisfied;
end