function Tensegrity = GetStellaOctangula()
% Stella Octangular.
% see "Form-Finding of Nonregular Tensegrity Systems".
% see also "Form finding of tensegrity structures using finite elements and
% mathematical programming".

n_strut = 6;n_cable = 18;
n_node = 2*n_strut;
n_elem = n_strut + n_cable;

Tensegrity.dim = 3;
Tensegrity.n_strut = n_strut;
Tensegrity.n_cable = n_cable;
Tensegrity.n_node = n_node;
Tensegrity.n_elem = n_elem;

s = 1.0;r = s/sqrt(3); h = s/sqrt(6);
vertex = zeros(n_node,3);
vertex(1,:) = [-s/2,-r/2,h];
vertex(2,:) = [s/2,-r/2,h];
vertex(3,:) = [0,r,h];
vertex(4,:) = [0,-2*r,h];
vertex(5,:) = [s,r,h];
vertex(6,:) = [-s,r,h];
vertex(7,:) = [s,-r,-h];
vertex(8,:) = [-s,-r,-h];
vertex(9,:) = [0,2*r,-h];
vertex(10,:) = [0,-r,-h];
vertex(11,:) = [-s/2,r/2,-h];
vertex(12,:) = [s/2,r/2,-h];
Tensegrity.Node = vertex;% + random('Normal',0,10,size(vertex,1),size(vertex,2))

% topology.
elem = zeros(n_elem,3);
elem(1,:) = [4,1,201];
elem(2,:) = [4,10,201];
elem(3,:) = [4,11,101];
elem(4,:) = [5,2,201];
elem(5,:) = [5,12,201];
elem(6,:) = [5,10,101];
elem(7,:) = [6,3,201];
elem(8,:) = [6,11,201];
elem(9,:) = [6,12,101];
elem(10,:) = [7,2,201];
elem(11,:) = [7,10,201];
elem(12,:) = [7,1,101];
elem(13,:) = [8,1,201];
elem(14,:) = [8,11,201];
elem(15,:) = [8,3,101];
elem(16,:) = [9,3,201];
elem(17,:) = [9,12,201];
elem(18,:) = [9,2,101];
elem(19,:) = [1,2,201];
elem(20,:) = [2,3,201];
elem(21,:) = [3,1,201];
elem(22,:) = [10,11,201];
elem(23,:) = [11,12,201];
elem(24,:) = [12,10,201];

% assign node dofs and element dofs.
ndof = zeros(n_node,3);
for i = 1:n_node
    ndof(i,:) = [3*i-2,3*i-1,3*i];
end
Tensegrity.node_dof = ndof;

Tensegrity.Elem = elem(:,1:2);
edof = zeros(n_elem,6);
for i = 1:n_elem
    for j = 1:2
        ic1 = 3*j - 2;ic2 = 3*j - 1;ic3 = 3*j;
        edof(i,ic1) = Tensegrity.Elem(i,j)*3 - 2;
        edof(i,ic2) = Tensegrity.Elem(i,j)*3 - 1;
        edof(i,ic3) = Tensegrity.Elem(i,j)*3;
    end
end
Tensegrity.elem_dof = edof;

c_strut = 0;strut_index = zeros(n_strut,1);strut_group = zeros(n_strut,1);
c_cable = 0;cable_index = zeros(n_cable,1);cable_group = zeros(n_cable,1);
for i = 1:n_elem
    if (elem(i,3) < 200)
        c_strut = c_strut + 1;
        strut_index(c_strut,1) = i;
        strut_group(c_strut,1) = elem(i,3)-100;
    elseif (elem(i,3) > 200)
        c_cable = c_cable + 1;
        cable_index(c_cable,1) = i;
        cable_group(c_cable,1) = elem(i,3)-200;
    end
end

Tensegrity.strut_index = strut_index;
Tensegrity.cable_index = cable_index;
Tensegrity.strut_group = strut_group;
Tensegrity.cable_group = cable_group;

% natural length, cross-section area and strain-modulus relationship for each group of struts.
strut_group_id = 1;
Tensegrity.strut_group_physical(strut_group_id).length = 2.5*sqrt(3)*s;
Tensegrity.strut_group_physical(strut_group_id).area = 1.0e-5;
Tensegrity.strut_group_physical(strut_group_id).rho = 7.80e3;
Tensegrity.strut_group_physical(strut_group_id).strain = [-1.0,0,1.0];
Tensegrity.strut_group_physical(strut_group_id).modulus = [2.0e11,2.0e11,2.0e11];
% natural length, cross-section area and strain-modulus relationship for each group of cables.
cable_group_id = 1;
Tensegrity.cable_group_physical(cable_group_id).length = 0.8*s;
Tensegrity.cable_group_physical(cable_group_id).area = 1.0e-5;
Tensegrity.cable_group_physical(cable_group_id).rho = 7.80e3;
Tensegrity.cable_group_physical(cable_group_id).strain = [-1.0,0,1.0];
Tensegrity.cable_group_physical(cable_group_id).modulus = [2.0e9,2.0e9,2.0e9];

for ie = 1:n_strut
    Tensegrity.L0(strut_index(ie)) = Tensegrity.strut_group_physical(Tensegrity.strut_group(ie)).length;
end
for ie = 1:n_cable
    Tensegrity.L0(cable_index(ie)) = Tensegrity.cable_group_physical(Tensegrity.cable_group(ie)).length;
end

scale = 1;
for i = 1:length(Tensegrity.strut_group_physical)
    index = find(0==Tensegrity.strut_group_physical(i).strain,'1', 'last');
    temp = Tensegrity.strut_group_physical(i).modulus(index)*Tensegrity.strut_group_physical(i).area;
    if (temp > scale)
        scale = temp;
    end
end
for i = 1:length(Tensegrity.cable_group_physical)
    index = find(0==Tensegrity.cable_group_physical(i).strain,'1', 'last');
    temp = Tensegrity.cable_group_physical(i).modulus(index)*Tensegrity.cable_group_physical(i).area;
    if (temp > scale)
        scale = temp;
    end
end
Tensegrity.scale = scale;

end