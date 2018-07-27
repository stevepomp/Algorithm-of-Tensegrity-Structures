function Model = Tensegrity2Model(Tensegrity, DOF_Active)
% tensegrity information transform to model information.
% common information.

Model.dim = Tensegrity.dim;
Model.n_node = Tensegrity.n_node;
Model.n_elem = Tensegrity.n_elem;
Model.n_strut = Tensegrity.n_strut;
Model.n_cable = Tensegrity.n_cable;
Model.Node = Tensegrity.Node;
Model.Elem = Tensegrity.Elem;
Model.strut_index = Tensegrity.strut_index;
Model.cable_index = Tensegrity.cable_index;
Model.strut_group = Tensegrity.strut_group;
Model.cable_group = Tensegrity.cable_group;
Model.strut_group_physical = Tensegrity.strut_group_physical;
Model.cable_group_physical = Tensegrity.cable_group_physical;

Model.ndofs = 0;
Model.node_dof = zeros(Model.n_node,3);
for i = 1:Model.n_node
    for k = 1:3
        if (DOF_Active(i,k)~=0)
            Model.ndofs = Model.ndofs + 1;
            Model.node_dof(i,k) = Model.ndofs;
        end
    end
end
Model.elem_dof = zeros(Model.n_elem,6);
for ie = 1:Model.n_elem
    Model.elem_dof(ie,1:3) = Model.node_dof(Model.Elem(ie,1),1:3);
    Model.elem_dof(ie,4:6) = Model.node_dof(Model.Elem(ie,2),1:3);
end

end