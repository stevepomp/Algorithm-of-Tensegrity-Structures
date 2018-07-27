function res = GetResult(Model, qbar)
% get the result of current load step.

res.node = zeros(Model.n_node,3);
for i = 1:Model.n_node
    node = Model.Node(i,:);
    for j = 1:Model.dim
        if (Model.node_dof(i,j)~=0)
            res.node(i,j) = qbar(Model.node_dof(i,j));
        else
            res.node(i,j) = node(j);
        end
    end
end

for ie = 1:Model.n_strut
    e = Model.strut_index(ie);
    pt1 = zeros(3,1); pt2 = zeros(3,1);
    for i = 1:Model.dim
        if (Model.node_dof(Model.Elem(e,1),i)~=0)
            pt1(i) = qbar(Model.node_dof(Model.Elem(e,1),i));
        else
            pt1(i) = Model.Node(Model.Elem(e,1),i);
        end
        if (Model.node_dof(Model.Elem(e,2),i)~=0)
            pt2(i) = qbar(Model.node_dof(Model.Elem(e,2),i));
        else
            pt2(i) = Model.Node(Model.Elem(e,2),i);
        end
    end
    l = sqrt((pt2(1)-pt1(1))^2+(pt2(2)-pt1(2))^2+(pt2(3)-pt1(3))^2);
    physical = Model.strut_group_physical(Model.strut_group(ie));
    l0 = physical.length;
    strain = l/l0-1;
    stress = GetStress(physical, strain);
    res.strain(e) = strain;
    res.stress(e) = stress;
end

for ie = 1:Model.n_cable
    e = Model.cable_index(ie);
    pt1 = zeros(3,1); pt2 = zeros(3,1);
    for i = 1:Model.dim
        if (Model.node_dof(Model.Elem(e,1),i)~=0)
            pt1(i) = qbar(Model.node_dof(Model.Elem(e,1),i));
        else
            pt1(i) = Model.Node(Model.Elem(e,1),i);
        end
        if (Model.node_dof(Model.Elem(e,2),i)~=0)
            pt2(i) = qbar(Model.node_dof(Model.Elem(e,2),i));
        else
            pt2(i) = Model.Node(Model.Elem(e,2),i);
        end
    end
    l = sqrt((pt2(1)-pt1(1))^2+(pt2(2)-pt1(2))^2+(pt2(3)-pt1(3))^2);
    physical = Model.cable_group_physical(Model.cable_group(ie));
    l0 = physical.length;
    if (l0 < l)
        strain = l/l0-1;
        stress = GetStress(physical, strain);
    else
        strain = 0;stress = 0;
    end
    res.strain(e) = strain;
    res.stress(e) = stress;
end

end