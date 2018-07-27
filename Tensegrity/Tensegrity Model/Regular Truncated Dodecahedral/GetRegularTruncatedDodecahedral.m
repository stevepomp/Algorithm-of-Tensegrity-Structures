function Tensegrity = GetRegularTruncatedDodecahedral(b, a)
% Regular Truncated Dodecahedral.
% b --- the edge length of a regular dodecahedral.
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
phi = 2*pi/3;
e0 = cos(phi/2);e1 = (sqrt(5)-1)/(2*sqrt(3))*sin(phi/2);%e2 = 0;
e3 = (sqrt(5)+1)/(2*sqrt(3))*sin(phi/2);
RG = 2*[e0^2+e1^2-0.5, -e0*e3, e1*e3;
    e0*e3, e0^2-0.5, -e0*e1;
    e1*e3, e0*e1, e0^2+e3^2-0.5];
FG = [-1, 0, 0;
    0, -1, 0;
    0, 0, 1];

vertex = zeros(n_node,3);
vertex(1,:) = b*[0.5-h, 0, (sqrt(5)+3)/4];
vertex(2,:) = b*[0.5+h*(sqrt(5)-1)/4, -h*(sqrt(5)+1)/4, (sqrt(5)+3)/4-h/2];
vertex(3,:) = b*[0.5+h*(sqrt(5)-1)/4, h*(sqrt(5)+1)/4, (sqrt(5)+3)/4-h/2];
for i = 1:3
    vertex(3+i,:) = FG*vertex(i,:)';
    vertex(6+i,:) = RG*vertex(3+i,:)';
    vertex(9+i,:) = RG*vertex(6+i,:)';
    vertex(12+i,:) = FG*vertex(6+i,:)';
    vertex(15+i,:) = RG*vertex(12+i,:)';
    vertex(18+i,:) = RG*vertex(15+i,:)';
    vertex(21+i,:) = FG*vertex(9+i,:)';
    vertex(24+i,:) = RG*vertex(21+i,:)';
    vertex(27+i,:) = RG*vertex(24+i,:)';
    vertex(30+i,:) = FG*vertex(24+i,:)';
    vertex(33+i,:) = RG*vertex(30+i,:)';
    vertex(36+i,:) = RG*vertex(33+i,:)';
    vertex(39+i,:) = FG*vertex(18+i,:)';
    vertex(42+i,:) = RG*vertex(39+i,:)';
    vertex(45+i,:) = RG*vertex(42+i,:)';
    vertex(48+i,:) = FG*vertex(42+i,:)';
    vertex(51+i,:) = RG*vertex(48+i,:)';
    vertex(54+i,:) = RG*vertex(51+i,:)';
    vertex(57+i,:) = FG*vertex(54+i,:)';
end
Tensegrity.Node = vertex;% + random('Normal',0,10,size(vertex,1),size(vertex,2))

% topology.
elem = zeros(n_elem,3);
% 30 struts.
for i = 1:3
    elem(3+i,:) = [1+3*i,12+3*i,101];
    elem(6+i,:) = [2+3*i,21+3*i,101];
    elem(12+i,:) = [11+3*i,30+3*i,101];
    elem(15+i,:) = [28+3*i,48+3*i,101];
    elem(18+i,:) = [29+3*i,37+3*i,101];
    elem(21+i,:) = [19+3*i,39+3*i,101];
end
for i = 1:2
    elem(i,:) = [i,6+3*i,101];
    elem(9+i,:) = [13+3*i,20+3*i,101];
    elem(24+i,:) = [38+3*i,49+3*i,101];
    elem(27+i,:) = [50+3*i,60-i+1,101];
end
elem(3,:) = [3,6,101];
elem(9+3,:) = [13,29,101];
elem(24+3,:) = [47,49,101];
elem(27+3,:) = [50,58,101];
% 60 cables consisting of pentagons.
for j = 1:20
    for i = 1:2
        elem(30+3*(j-1)+i,:) = [3*(j-1)+i,3*(j-1)+i+1,201];
    end
    elem(30+3*(j-1)+3,:) = [3*(j-1)+3,3*(j-1)+1,201];
end
% 30 cables connecting the pentagons.
for i = 1:3
    elem(90+i,:) = [i,3*i+1,202];
    elem(93+i,:) = [3+3*i,19+3*i,202];
    elem(96+i,:) = [2+3*i,10+3*i,202];
    elem(102+i,:) = [37+3*i,20+3*i,202];
    elem(105+i,:) = [28+3*i,12+3*i,202];
    elem(108+i,:) = [30+3*i,38+3*i,202];
    elem(111+i,:) = [46+3*i,29+3*i,202];
end
for i = 1:2
    elem(99+i,:) = [21+3*i,14+3*i,202];
    elem(114+i,:) = [39+3*i,50+3*i,202];
    elem(117+i,:) = [48+3*i,60-i,202];
end
elem(99+3,:) = [21+3*3,14,202];
elem(114+3,:) = [39+3*3,50,202];
elem(117+3,:) = [48+3*3,60,202];

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