function [A, D] = AssemblyAD(Tensegrity)
% Assembly for post analysis.

[L, DirV, Fint] = GetState(Tensegrity);

n_node = Tensegrity.n_node;
n_elem = Tensegrity.n_elem;
dim = Tensegrity.dim;
edof = Tensegrity.elem_dof;

D = zeros(n_node,n_node);
for i = 1:n_elem
    qe = Fint(i)/L(i);
    ind1 = Tensegrity.Elem(i,1);
    ind2 = Tensegrity.Elem(i,2);
    D(ind1,ind2) = - qe;
    D(ind2,ind1) = - qe;
end
for i = 1:n_node
    for k = 1:n_node
        if (k~=i)
            D(i,i) = D(i,i) - D(i,k);
        end
    end
end

A = zeros(dim*n_node,n_elem);
for i = 1:n_elem
    ke = [DirV(:,i);-DirV(:,i)]*L(i);
    for j = 1:2*dim
        A(edof(i,j),i) = A(edof(i,j),i) + ke(j);
    end
end

end
