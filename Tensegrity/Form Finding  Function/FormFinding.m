function Tensegrity = FormFinding(Tensegrity)
% Find the self-stressed equilibrium state and update Tensegrity.
% Newton-Raphson iteration is used.
% 

% calculate the initial state.
dim = 3;n_node = Tensegrity.n_node;
q = zeros(dim*n_node,1);% node coordinates, row vector.
for i = 1:n_node
    q(dim*i-(dim-1):dim*i) = Tensegrity.Node(i,1:dim);
end

% calculate the stiffness matrix and out-of balance force vector.
[K, F] = Assembly(Tensegrity);

epsilon = max(abs(F));

% set the iteration step.
N = 0;
% iteration.
epsilon_threshold = 1.0e-11;xi = 1.01;
while(epsilon > epsilon_threshold)
    % constraint rigid-body DOFs.
    [Kbar, Fbar] = Constraint(K, F, n_node, dim);
    % trust region/restricted step algorithm.
    [Ktilde] = TrustRegion(Kbar, xi, n_node, dim);
    % solve the nodal displacements.
    Dq = Ktilde\Fbar;
    % backtracking line search algorithm.
    m = Dq'*F;
    alpha = BacktrackingLineSearch(Tensegrity, q, Dq, m);
    % update node coordinates.
    q_new = q + alpha*Dq;
    for i = 1:n_node
        Tensegrity.Node(i,1:dim) = q_new(dim*i-(dim-1):dim*i);
    end
    % calculate the stiffness matrix and out-of balance force vector.
    [K, F_new] = Assembly(Tensegrity);
    % epsilon = max(abs(F));
    epsilon = norm(F,2);
    % update iteration.
    N = N + 1;
    if (N > 5000)
        epsilon_threshold = 1.0e-6;
    elseif (N > 2000)
        epsilon_threshold = 1.0e-8;
    elseif (N > 1000)
        epsilon_threshold = 1.0e-10;
    end
    F = F_new;
    q = q_new;
    %disp(N);
    fprintf('Iteration = %d, Convergence = %f\n', N, log10(epsilon));
    Tensegrity.Residual(N,:) = [N,log10(epsilon)];
    if (N>10000)
        return;
    end
end

end

% function Tensegrity = AffineTransform2D(Tensegrity)
% 
% % get node 1, 2, and 3.
% node1 = Tensegrity.Node(1,:);
% node2 = Tensegrity.Node(2,:);
% 
% % Firstly, node-1 as a reference point. Then, the unit vector vx is the unit
% % direction vector point from node-1 to node-2. 
% v12 = node2-node1; v12 = v12/norm(v12); v13 = [0,0,1];
% vz = cross(v12,v13);vz = vz/norm(vz);
% vx = v12;vy = cross(vz,vx);
% 
% translation = node1;
% rotation = [vx;vy;vz]^-1;
% n_node = Tensegrity.n_node;
% % translation and rotaion Tensegrity.
% Tensegrity.Node = Tensegrity.Node - repmat(translation,n_node,1);
% Tensegrity.Node = Tensegrity.Node*rotation;
% 
% end

