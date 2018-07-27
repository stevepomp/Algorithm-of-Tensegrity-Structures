function [L, DirV, Fint, Stiffness] = GetState(Tensegrity)
% Get the curretn state of Tensegrity.
% L: element length, column vector.
% DirV: element direction vector, matrix.
% Fint: element internal force, row vector.
% 

dim = 3;
n_elem = Tensegrity.n_elem;
n_strut = Tensegrity.n_strut;
n_cable = Tensegrity.n_cable;
strut_index = Tensegrity.strut_index;
cable_index = Tensegrity.cable_index;

L = zeros(1,n_elem);
DirV = zeros(dim,n_elem);
Fint = zeros(1,n_elem);
L0 = Tensegrity.L0;
for ie = 1:n_strut
    e = strut_index(ie);
    node1 = Tensegrity.Node(Tensegrity.Elem(e,1),:);
    node2 = Tensegrity.Node(Tensegrity.Elem(e,2),:);
    L(e) = norm(node1-node2)+eps;% avoid NaN or Inf.
    DirV(:,e) = ((node1-node2)/L(e))';
    Fint(e) = (L(e)-L0(e));
end

for ie = 1:n_cable
    e = cable_index(ie);
    node1 = Tensegrity.Node(Tensegrity.Elem(e,1),:);
    node2 = Tensegrity.Node(Tensegrity.Elem(e,2),:);
    L(e) = norm(node1-node2)+eps;% avoid NaN or Inf.
    DirV(:,e) = ((node1-node2)/L(e))';
    if (L(e) > L0(e))
        Fint(e) = (L(e)-L0(e));
    else
        L(e) = L0(e);
    end
end

Stiffness = GetStiffness(Tensegrity, L);
for e = 1:n_elem
    Fint(e) = Fint(e)*Stiffness(e);
end

end