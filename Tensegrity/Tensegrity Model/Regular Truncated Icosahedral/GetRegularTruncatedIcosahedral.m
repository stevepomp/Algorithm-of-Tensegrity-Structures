function Tensegrity = GetRegularTruncatedIcosahedral(b, a)
% Regular Truncated Icosahedral.
% b --- the edge length of a regular icosahedron.
% a --- the length of the truncated edges, a<b/2.
% h --- the truncated ratio, h = a/b.
% alpha --- twist angle.
% phi --- Euler parameters, phi = 2*pi/5.

n_strut = 30;n_cable = 90;
n_node = 2*n_strut;
n_elem = n_strut + n_cable;

Tensegrity.dim = 3;
Tensegrity.n_strut = n_strut;
Tensegrity.n_cable = n_cable;
Tensegrity.n_node = n_node;
Tensegrity.n_elem = n_elem;

h = a/b;
phi = 2*pi/5;
e0 = cos(phi/2);e1 = sqrt(2)/(sqrt(5+sqrt(5)))*sin(phi/2);%e2 = 0;
e3 = (1+sqrt(5))/(sqrt(2)*(sqrt(5+sqrt(5))))*sin(phi/2);
RG = 2*[e0^2+e1^2-0.5, -e0*e3, e1*e3;
    e0*e3, e0^2-0.5, -e0*e1;
    e1*e3, e0*e1, e0^2+e3^2-0.5];
FG = [-1, 0, 0;
    0, -1, 0;
    0, 0, 1];

vertex = zeros(n_node,3);
vertex(1,:) = b*[0.5-h, 0, (sqrt(5)+1)/4];
for i = 2:5
    vertex(i,:) = RG*vertex(i-1,:)';
end
for i = 1:5
    vertex(5+i,:) = FG*vertex(i,:)';
end
for j = 2:5
    for i = 1:5
        vertex(5*j+i,:) = RG*vertex(5*(j-1)+i,:)';
    end
end
for i = 1:5
    vertex(30+i,:) = FG*vertex(20+i,:)';
end
for j = 1:4
    for i = 1:5
        vertex(30+5*j+i,:) = RG*vertex(30+5*(j-1)+i,:)';
    end
end
for i = 1:5
    vertex(55+i,:) = FG*vertex(40+i,:)';
end
Tensegrity.Node = vertex;% + random('Normal',0,10,size(vertex,1),size(vertex,2))

% topology.
% cable: f-e+v=2 => 32-e+60=2 => e=90.
% strut: 12*5/2=30.
elem = zeros(n_elem,3);
% 60 cales consisting of pentagons.
for j = 1:12
    for i = 1:4
        elem(5*(j-1)+i,:) = [5*(j-1)+i,5*(j-1)+i+1,201];
    end
    elem(5*(j-1)+5,:) = [5*(j-1)+5,5*(j-1)+1,201];
end
% 30 cables connecting the pentagons.
for i = 1:5
    elem(60+i,:) = [i,5*i+1,202];
    elem(70+i,:) = [4+5*i,26+5*i,202];
    elem(85+i,:) = [28+5*i,60-i+1,202];
end
for i = 1:4
    elem(65+i,:) = [5+5*i,7+5*i,202];
    elem(75+i,:) = [8+5*i,30+5*i,202];
    elem(80+i,:) = [29+5*i,32+5*i,202];
end
elem(65+5,:) = [5+5*5,7,202];
elem(75+5,:) = [8,30+5*5,202];
elem(80+5,:) = [29+5*5,32,202];
% strut.
for i = 1:4
    elem(90+i,:) = [i,10+5*i,101];
    elem(95+i,:) = [4+5*i,6+5*i,101];
    elem(105+i,:) = [7+5*i,29+5*i,101];
    elem(110+i,:) = [28+5*i,31+5*i,101];
    elem(115+i,:) = [27+5*i,60-i,101];
end
elem(90+5,:) = [5,10,101];
elem(95+5,:) = [4+5*5,6,101];
elem(105+5,:) = [7,29+5*5,101];
elem(110+5,:) = [28+5*5,31,101];
elem(115+5,:) = [27+5*5,60,101];
for i = 1:5
    elem(100+i,:) = [3+5*i,30+5*i,101];
end

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
Tensegrity.strut_group_physical(strut_group_id).length = b;
Tensegrity.strut_group_physical(strut_group_id).area = 1.0e-5;
Tensegrity.strut_group_physical(strut_group_id).rho = 7.80e3;
Tensegrity.strut_group_physical(strut_group_id).strain = [-1.0,0,1.0];
Tensegrity.strut_group_physical(strut_group_id).modulus = [2.0e11,2.0e11,2.0e11];
% natural length, cross-section area and strain-modulus relationship for each group of cables.
cable_group_id = 1;
Tensegrity.cable_group_physical(cable_group_id).length = a;
Tensegrity.cable_group_physical(cable_group_id).area = 1.0e-5;
Tensegrity.cable_group_physical(cable_group_id).rho = 7.80e3;
Tensegrity.cable_group_physical(cable_group_id).strain = [-1.0,0,1.0];
Tensegrity.cable_group_physical(cable_group_id).modulus = [2.0e11,2.0e11,2.0e11];
cable_group_id = 2;
Tensegrity.cable_group_physical(cable_group_id).length = (b-2*a);
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