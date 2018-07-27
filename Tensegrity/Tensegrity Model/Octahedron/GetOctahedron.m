function Tensegrity = GetOctahedron()
% Octahedron tensegrity.

n_strut = 6;n_cable = 24;
n_node = 2*n_strut;
n_elem = n_strut + n_cable;

Tensegrity.dim = 3;
Tensegrity.n_strut = n_strut;
Tensegrity.n_cable = n_cable;
Tensegrity.n_node = n_node;
Tensegrity.n_elem = n_elem;

rad = 1.0;
K = [1/16*(10+2*sqrt(5)), 0, 0, 0;
    0, 1/3, 1, 0;
    -1, 1/3, 4, 0;
    1/3, 0, 0, 1];
F = [rad, rad, 0, rad]';
dof = K\F;
a = sqrt(dof(1));
b = sqrt(dof(2));
g = sqrt(dof(3));
h = sqrt(dof(4));
fac1 = sqrt(3)/3;
fac2 = sqrt(3)/6;
q0 =[0; +fac1*a; -h;
    -a/2; -fac2*a; -h;
    +a/2; -fac2*a; -h;
    0; -fac1*b; -g;
    +b/2; +fac2*b; -g;
    -b/2; +fac2*b; -g;
    0; +fac1*b; +g;
    -b/2; -fac2*b; +g;
    +b/2; -fac2*b; +g;
    0; -fac1*a; +h;
    +a/2; +fac2*a; +h;
    -a/2; +fac2*a; +h];
vertex = zeros(n_node,3);
for i = 1:n_node
    vertex(i,:) = q0(3*i-2:3*i);
end
Tensegrity.Node = vertex;% + random('Normal',0,10,size(vertex,1),size(vertex,2))

% topology of the tensegrity.
% struts
elem(1,:) = [1 9 101];
elem(2,:) = [2 7 101];
elem(3,:) = [3 8 101];
elem(4,:) = [4 11 101];
elem(5,:) = [5 12 101];
elem(6,:) = [6 10 101];

% cables
elem(7,:) = [1 2 201];
elem(8,:) = [2 3 201];
elem(9,:) = [3 1 201];
elem(10,:) = [10 11 201];
elem(11,:) = [11 12 201];
elem(12,:) = [12 10 201];

elem(13,:) = [1 5 202];
elem(14,:) = [2 6 202];
elem(15,:) = [3 4 202];
elem(16,:) = [10 8 202];
elem(17,:) = [11 9 202];
elem(18,:) = [12 7 202];

elem(19,:) = [1 7 203];
elem(20,:) = [2 8 203];
elem(21,:) = [3 9 203];
elem(22,:) = [10 4 203];
elem(23,:) = [11 5 203];
elem(24,:) = [12 6 203];

elem(25,:) = [4 9 204];
elem(26,:) = [9 5 204];
elem(27,:) = [5 7 204];
elem(28,:) = [7 6 204];
elem(29,:) = [6 8 204];
elem(30,:) = [8 4 204];

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
Tensegrity.strut_group_physical(strut_group_id).length = 1.0;
Tensegrity.strut_group_physical(strut_group_id).area = 1.9e-16;
Tensegrity.strut_group_physical(strut_group_id).rho = 7.80e3;
Tensegrity.strut_group_physical(strut_group_id).strain = [-1.0,0,1.0];
Tensegrity.strut_group_physical(strut_group_id).modulus = [1.2e9,1.2e9,1.2e9];
% natural length, cross-section area and strain-modulus relationship for each group of cables.
cable_group_id = 1;
Tensegrity.cable_group_physical(cable_group_id).length = 0.536;
Tensegrity.cable_group_physical(cable_group_id).area = 1.8e-17;
Tensegrity.cable_group_physical(cable_group_id).rho = 7.80e3;
Tensegrity.cable_group_physical(cable_group_id).strain = [-1.0,0,1.0];
Tensegrity.cable_group_physical(cable_group_id).modulus = [2.6e5,2.6e5,2.6e5];
cable_group_id = 2;
Tensegrity.cable_group_physical(cable_group_id).length = 0.536;
Tensegrity.cable_group_physical(cable_group_id).area = 1.8e-17;
Tensegrity.cable_group_physical(cable_group_id).rho = 7.80e3;
Tensegrity.cable_group_physical(cable_group_id).strain = [-1.0,0,1.0];
Tensegrity.cable_group_physical(cable_group_id).modulus = [2.6e5,2.6e5,2.6e5];
cable_group_id = 3;
Tensegrity.cable_group_physical(cable_group_id).length = 0.536;
Tensegrity.cable_group_physical(cable_group_id).area = 1.8e-17;
Tensegrity.cable_group_physical(cable_group_id).rho = 7.80e3;
Tensegrity.cable_group_physical(cable_group_id).strain = [-1.0,0,1.0];
Tensegrity.cable_group_physical(cable_group_id).modulus = [2.6e5,2.6e5,2.6e5];
cable_group_id = 4;
Tensegrity.cable_group_physical(cable_group_id).length = 0.536;
Tensegrity.cable_group_physical(cable_group_id).area = 1.8e-17;
Tensegrity.cable_group_physical(cable_group_id).rho = 7.80e3;
Tensegrity.cable_group_physical(cable_group_id).strain = [-1.0,0,1.0];
Tensegrity.cable_group_physical(cable_group_id).modulus = [2.6e5,2.6e5,2.6e5];

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