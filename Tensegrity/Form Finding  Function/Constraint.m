function [Kbar, Fbar] = Constraint(K, F, n_node, dim)
% Stochastic selecting algorithm.

Constraint_DOF = [1,2,3,5,6,9];
Kbar = K;Fbar = F;
for j = 1:3*dim-3
    J = Constraint_DOF(j);%
    Kbar(J,J) = 1;
    Fbar(J) = 0;
    for L = 1:dim*n_node
        if (J ~= L)
            Kbar(J,L) = 0;
            Kbar(L,J) = 0;
        end
    end
end

end