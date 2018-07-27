function Tensegrity = GetAntiPrismaticTensegrity(v, a, lc1, lc2, lc3, lc4, lb)
% example
% case 1
% v = 3;
% a = 1;
% lc1 = 1.0;
% lc2 = 1.0;
% lc3 = 1.5;
% lc4 = 1.6;
% lb = 2.0;

% number of struts, cables, and nodes.
n_strut = v;n_cable = 4*v;
n_node = 2*v;n_elem = n_strut + n_cable;

Tensegrity.dim = 3;
Tensegrity.n_strut = n_strut;
Tensegrity.n_cable = n_cable;
Tensegrity.n_node = n_node;
Tensegrity.n_elem = n_elem;

% coordinates of nodes.
for i = 1:v
    Tensegrity.Node(i,:) = [cos(i/v*2*pi) sin(i/v*2*pi) 0.0];
    Tensegrity.Node(i+v,:) = [cos(i/v*2*pi) sin(i/v*2*pi) 1.0];
end
Tensegrity.Node = Tensegrity.Node;% + random('Normal',0,1,size(Tensegrity.Node,1),size(Tensegrity.Node,2))
% topology of tensegrity structures.
elem = zeros(n_elem,3);
% a: the offset between vertices connected by strut.
% struts
for i = 1:v-a
    elem(i,:) = [i,i+v+a,101];
end
for i = v-a+1:v
    elem(i,:) = [i,i-v+a+v,101];
end
% cables
for i = 1:v-1
    elem(i+v,:) = [i+v,i+1+v,201];% qs1, top, red.
    elem(i+2*v,:) = [i,i+1,202];% qs2, bottom, blue.
    elem(i+3*v,:) = [i,i+v,203];% qs3, lateral, green.
    elem(i+4*v,:) = [i+v,i+1,204];% qs4, additional, cyan.
end
elem(v+v,:) = [v+v,v+1,201];% qs1
elem(v+2*v,:) = [v,1,202];% qs2
elem(v+3*v,:) = [v,v+v,203];% qs3
elem(v+4*v,:) = [v+v,1,204];% qs4, added cable

% assign node dofs and element dofs.
ndof = zeros(n_node,3);
for i = 1:n_node
    ndof(i,:) = [3*i-2,3*i-1,3*i];
end

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

Tensegrity.node_dof = ndof;
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
Tensegrity.strut_group_physical(strut_group_id).length = lb;
Tensegrity.strut_group_physical(strut_group_id).area = 5.0e-5;
Tensegrity.strut_group_physical(strut_group_id).rho = 7.80e3;
Tensegrity.strut_group_physical(strut_group_id).strain = [-1.0,0,1.0];
Tensegrity.strut_group_physical(strut_group_id).modulus = [2.0e11,2.0e11,2.0e11];
% natural length, cross-section area and strain-modulus relationship for each group of cables.
cable_group_id = 1;
Tensegrity.cable_group_physical(cable_group_id).length = lc1;
Tensegrity.cable_group_physical(cable_group_id).area = 1.0e-5;
Tensegrity.cable_group_physical(cable_group_id).rho = 7.80e3;
Tensegrity.cable_group_physical(cable_group_id).strain = [-1.0,0,1.0];
Tensegrity.cable_group_physical(cable_group_id).modulus = [2.0e11,2.0e11,2.0e11];
cable_group_id = 2;
Tensegrity.cable_group_physical(cable_group_id).length = lc2;
Tensegrity.cable_group_physical(cable_group_id).area = 1.0e-5;
Tensegrity.cable_group_physical(cable_group_id).rho = 7.80e3;
Tensegrity.cable_group_physical(cable_group_id).strain = [-1.0,0,1.0];
Tensegrity.cable_group_physical(cable_group_id).modulus = [2.0e11,2.0e11,2.0e11];
cable_group_id = 3;
Tensegrity.cable_group_physical(cable_group_id).length = lc3;
Tensegrity.cable_group_physical(cable_group_id).area = 1.0e-5;
Tensegrity.cable_group_physical(cable_group_id).rho = 7.80e3;
Tensegrity.cable_group_physical(cable_group_id).strain = [-1.0,0,1.0];
Tensegrity.cable_group_physical(cable_group_id).modulus = [2.0e11,2.0e11,2.0e11];
cable_group_id = 4;
Tensegrity.cable_group_physical(cable_group_id).length = lc4;
Tensegrity.cable_group_physical(cable_group_id).area = 1.0e-5;
Tensegrity.cable_group_physical(cable_group_id).rho = 7.80e3;
Tensegrity.cable_group_physical(cable_group_id).strain = [-1.0,0,1.0];
Tensegrity.cable_group_physical(cable_group_id).modulus = [2.0e11,2.0e11,2.0e11];

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