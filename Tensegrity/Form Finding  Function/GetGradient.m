function grad = GetGradient(Tensegrity)
% Get the gradient of the objective function.

[L, DirV, Fint] = GetState(Tensegrity);

dim = Tensegrity.dim;n_node = Tensegrity.n_node;
n_elem = Tensegrity.n_elem;edof = Tensegrity.edof;
grad = zeros(dim*n_node,1);
for i = 1:n_elem
    fe = Fint(i)*[-DirV(:,i);DirV(:,i)];
    for j = 1:2*dim
        grad(edof(i,j)) = grad(edof(i,j)) + fe(j);
    end
end

end