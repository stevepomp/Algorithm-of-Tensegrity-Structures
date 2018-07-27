function [K, F] = Assembly(Tensegrity)
% assembly.

[L, DirV, Fint, Stiffness] = GetState(Tensegrity);

dim = 3;n_node = Tensegrity.n_node;n_elem = Tensegrity.n_elem;
L0 = Tensegrity.L0;edof = Tensegrity.elem_dof;

K = zeros(dim*n_node,dim*n_node);F = zeros(dim*n_node,1);
IK = -diag(ones(dim,1),-dim) + diag(ones(2*dim,1),0) - diag(ones(dim,1),dim);
for i = 1:n_elem
    ke = Fint(i)/L(i)*IK + Stiffness(i)*L0(i)/L(i)*([DirV(:,i);-DirV(:,i)]*[DirV(:,i);-DirV(:,i)]');
    fe = Fint(i)*[-DirV(:,i);DirV(:,i)];
    for j = 1:2*dim
        for k = 1:2*dim
            K(edof(i,j),edof(i,k)) = K(edof(i,j),edof(i,k)) + ke(j,k);
        end
        F(edof(i,j)) = F(edof(i,j)) + fe(j);
    end
end

% eij = zeros(n_node,n_node);
% for i = 1:n_elem
%     eij(Tensegrity.Elem(i,1),Tensegrity.Elem(i,2)) = i;
% end
% % neighbor lists.
% mu = zeros(n_node,1);% the total number of elements containing node i.
% for i = 1:n_node
%     for j = 1:n_node
%         if (eij(i,j) ~= 0)
%             mu(i) = mu(i)+1;
%             mu(j) = mu(j)+1;
%         end
%     end
% end
% MaxDim = max(mu);
% clear mu;
% 
% mu = zeros(n_node,1);
% M = zeros(n_node,MaxDim);
% O = zeros(n_node,MaxDim);
% for i = 1:n_node
%     for j = 1:n_node
%         if (eij(i,j) ~= 0)
%             mu(i) = mu(i)+1;
%             M(i,mu(i)) = eij(i,j);
%             O(i,mu(i)) = -1;
%             mu(j) = mu(j)+1;
%             M(j,mu(j)) = eij(i,j);
%             O(j,mu(j)) = 1;
%         end
%     end
% end
% 
% K = zeros(dim*n_node,dim*n_node);
% for i = 1:n_node
%     for j = 1:n_node
%         if (i ~= j)
%             if (eij(i,j) ~= 0)
%                 ke = Fint(eij(i,j))/L(eij(i,j))*eye(dim)+Sitffness(eij(i,j))*L0(eij(i,j))/L(eij(i,j))*(DirV(:,eij(i,j))*DirV(:,eij(i,j))');
%                 K(dim*i-(dim-1):dim*i,dim*j-(dim-1):dim*j) = - ke;
%                 K(dim*j-(dim-1):dim*j,dim*i-(dim-1):dim*i) = - ke;
%             end
%         end
%     end
% end
% for i = 1:n_node
%     for j = 1:n_node
%         if (j ~= i)
%             ke = K(dim*i-(dim-1):dim*i,dim*j-(dim-1):dim*j);
%             K(dim*i-(dim-1):dim*i,dim*i-(dim-1):dim*i) = K(dim*i-(dim-1):dim*i,dim*i-(dim-1):dim*i) - ke;
%         end
%     end
% end
% 
% F = zeros(dim*n_node,1);
% for i = 1:n_node
%     for I = 1:mu(i)
%         F(dim*i-(dim-1):dim*i,1) = F(dim*i-(dim-1):dim*i,1)+O(i,I)*Fint(M(i,I))*DirV(:,M(i,I));
%     end
% end

end
