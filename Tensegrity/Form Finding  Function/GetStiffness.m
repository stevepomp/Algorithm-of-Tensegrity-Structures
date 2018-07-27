function Stiffness = GetStiffness(Tensegrity, L)
% Get the stiffness of current state.

n_elem = length(L);
n_strut = Tensegrity.n_strut;
n_cable = Tensegrity.n_cable;
strut_index = Tensegrity.strut_index;
cable_index = Tensegrity.cable_index;

L0 = Tensegrity.L0;
Stiffness = zeros(1,n_elem);
for ie = 1:n_strut
    e = strut_index(ie);
    physical = Tensegrity.strut_group_physical(Tensegrity.strut_group(ie));
    strain = L(e)/L0(e)-1;% current strain.
    index = find(strain >= physical.strain, 1, 'last');
    Stiffness(e) = physical.modulus(index)*physical.area;
end

for ie = 1:n_cable
    e = cable_index(ie);
    physical = Tensegrity.cable_group_physical(Tensegrity.cable_group(ie));
    strain = L(e)/L0(e)-1;% current strain.
    index = find(strain >= physical.strain, 1, 'last');
    Stiffness(e) = physical.modulus(index)*physical.area;
end

Stiffness = Stiffness/Tensegrity.scale;

end