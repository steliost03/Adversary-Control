%solves the optimization problem :
%determine c,E such that max{c} while respecting the conditions :
% EQ <= W (1) , E(cq) <= r (2) , E >= O (3) , c < 1 (4) , -c < 0 (5)
%where : Mc = { z : Qz <= cq } and Hv : { z : Wz <= w}
%(Farka's Lemma : implies Mc(Q,cq) is a subset of Hv(W,w))
% E dimensions : 1Xqdim ,where qdim the number of rows of Q.
% Q and W column number must be 2 (special case for 2x2 systems)

function c = farkasoptim_mpt(q,Q,w,W)

    


end