function [current_stress] = GetStress(physical, current_strain)
% get current current_stress of element (e).

current_stress = 0;
index0 = find(physical.strain == 0, 1, 'last');
index = find(current_strain >= physical.strain, 1, 'last');
if (index >= index0)% non-negtive current_strain
    for i = index0:index-1
        current_stress = current_stress + physical.modulus(i)*(physical.strain(i+1)-physical.strain(i));
    end
    current_stress = current_stress + physical.modulus(index)*(current_strain-physical.strain(index));
else
    for i = index0:-1:index+2
        current_stress = current_stress + physical.modulus(i-1)*(physical.strain(i-1)-physical.strain(i));
    end
    current_stress = current_stress + physical.modulus(index)*(current_strain-physical.strain(index+1));
end

end