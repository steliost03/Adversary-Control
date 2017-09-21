%finds the limit points (edges) that define a facet of a polyhedral.
%takes the hyperplane induced by the facet as an input (Pi,ri)
%,as well as the polyhedral it belongs to in H-representation (P,r)
function facetLimitPoints = findFacetLimits(P,r,Pi,ri)
    
    dim = size(P,2);

    D_Area = Polyhedron('A',P,'b',r);
    
    %vertex enumeration (row-wise representation)
    D_V = D_Area.V;
    
    vertexCount = size(D_V,1);
    facetLimitPoints = zeros(dim,2);
    flp_count = 0; %(facet limit point count)
    
    D_V = transpose(D_V);
    for i=1:vertexCount
        currVertex = D_V(:,i);
        isPartOfFacet = (abs(Pi*currVertex - ri) < 0.01);
        if(isPartOfFacet)
            flp_count = flp_count + 1;
            facetLimitPoints(:,flp_count) = currVertex;
        end
    end 
    
end