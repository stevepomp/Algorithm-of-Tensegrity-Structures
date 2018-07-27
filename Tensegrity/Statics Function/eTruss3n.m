function [kt, kp] = eTruss3n(eposition, physical)
% elemental subroutine, 2-node truss element and each node has 3-dof.
% kt: elemental tangent stiffness matrix.
% kq: elemental internal force vector.
% 

pt1 = eposition(1:3); pt2 = eposition(4:6);
l0 = physical.length;area = physical.area;

l = sqrt((pt2(1)-pt1(1))^2+(pt2(2)-pt1(2))^2+(pt2(3)-pt1(3))^2);
strain = l/l0-1;% current strain.
index = find(strain >= physical.strain, 1, 'last');
emod = physical.modulus(index);% current modulus.
stress = GetStress(physical, strain);
phi = zeros(6,1);
for i = 1:3
    phi(i) = (pt1(i)-pt2(i))/(l*l0);
    phi(i+3) = -phi(i);
end
gamma = (-diag(ones(3,1),-3)+diag(ones(6,1),0)-diag(ones(3,1),3))/(l0*l)-l0/l*(phi*phi');
kt = emod*area*l0*(phi*phi')+stress*area*l0*gamma;
kp = stress*area*l0*phi;

end