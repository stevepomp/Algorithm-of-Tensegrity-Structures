function U = GetEnergy(Tensegrity, q)
% Calculate the total potential energy.

U = 0;L0 = Tensegrity.L0;

dim = Tensegrity.dim;
n_elem = Tensegrity.n_elem;
n_node = Tensegrity.n_node;
n_strut = Tensegrity.n_strut;
n_cable = Tensegrity.n_cable;
strut_index = Tensegrity.strut_index;
cable_index = Tensegrity.cable_index;

node = zeros(n_node,dim);
for ie = 1:n_node
    node(ie,1:dim) = q(dim*ie-(dim-1):dim*ie);
end

L = zeros(1,n_elem);
Ue = zeros(1,n_elem);
for ie = 1:n_strut
    e = strut_index(ie);
    node1 = node(Tensegrity.Elem(e,1),:);
    node2 = node(Tensegrity.Elem(e,2),:);
    L(e) = norm(node1-node2);
    Ue(e) = 0.5*(L(e)-L0(e))^2;
end

for ie = 1:n_cable
    e = cable_index(ie,1);
    node1 = node(Tensegrity.Elem(e,1),:);
    node2 = node(Tensegrity.Elem(e,2),:);
    L(e) = norm(node1-node2);
    if (L(e) > L0(e))
        Ue(e) = 0.5*(L(e)-L0(e))^2;
    end
end

Stiffness = GetStiffness(Tensegrity, L);
for e = 1:n_elem
    Ue(e) = Ue(e)*Stiffness(e);
    U = U + Ue(e);
end

end