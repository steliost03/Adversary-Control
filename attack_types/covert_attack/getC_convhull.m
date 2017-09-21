%assumes facet is represented as a half-space in the form : {x : Px <= r }
%G and w represent the alarm constraints.
function [C_out,c_out,facetLimits,totalpoints] = getC_convhull(G,w,P,r,x0,chosenFacetIndex)

    dim = size(P,2);
    alarmFacetCount = size(G,1);
    
    %find facet limit points
    facetLimits = findFacetLimits(P,r,P(chosenFacetIndex,:),r(chosenFacetIndex));
    facetLimits_tr = transpose(facetLimits);
    facetLimitCount = size(facetLimits_tr,1);
    
    %form convex hull of those points,compute H representation of the
    %convex hull.
    totalpoints = zeros(size(facetLimits_tr,1)+1,size(facetLimits_tr,2));
    for i=1:facetLimitCount
        totalpoints(i,:) = facetLimits_tr(i,:);
    end
    totalpoints(facetLimitCount+1,:) = x0;
    
    cone = Polyhedron(totalpoints);
    cone.computeHRep;
    inequalities = cone.H;
    
    inequalityCount = size(inequalities,1);
    C = zeros(inequalityCount,dim);
    c = zeros(inequalityCount,1);
    
    for i=1:inequalityCount
        for j=1:dim
            C(i,j) = inequalities(i,j);
        end
        c(i) = inequalities(i,dim+1);
    end
    
    %find inequality to remove
    targetInequalityIndex = 0;
    for i=1:inequalityCount
        if(not( abs(C(i,:)*x0 - c(i)) < 0.1))
            targetInequalityIndex = i;
            break;
        end
    end
    
    C_out = zeros(inequalityCount-1 + alarmFacetCount ,dim);
    c_out = zeros(inequalityCount+alarmFacetCount-1,1);

    %remove the target inequality
    coutCounter = 0;
    for i=1:inequalityCount
        if(not(i==targetInequalityIndex))
            coutCounter = coutCounter + 1;
            C_out(coutCounter,:) = C(i,:);
            c_out(coutCounter) = c(i);
        end
    end
    
    %intersection with alarm constraints
    total_cols = inequalityCount - 1 + alarmFacetCount;
    Gcounter = 0;
    for i=inequalityCount:total_cols
        Gcounter = Gcounter + 1;
        C_out(i,:) = G(Gcounter,:);
        c_out(i) = w(Gcounter);
    end
    
   
end