function NDResult = PostProcessDynamicsInfo(NDResult, Model)
% post process dynamics information.

nTime = numel(NDResult);
for iTime = 1:nTime
    NDResult(iTime).node = zeros(Model.n_node,3);
    NDResult(iTime).velocity = zeros(Model.n_node,3);
    NDResult(iTime).acceleration = zeros(Model.n_node,3);
    for i = 1:Model.n_node
        for j = 1:3
            if (Model.node_dof(i,j)~=0)
                NDResult(iTime).node(i,j) = NDResult(iTime).q(Model.node_dof(i,j));
                NDResult(iTime).velocity(i,j) = NDResult(iTime).qt(Model.node_dof(i,j));
                NDResult(iTime).acceleration(i,j) = NDResult(iTime).qtt(Model.node_dof(i,j));
            end
        end
    end
end

end