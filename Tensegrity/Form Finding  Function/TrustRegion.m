function [Ktilde] = TrustRegion(Kbar, xi, n_node, dim)
% Trust Region/Restricted Step Algorithm.

lambda = min(eig(Kbar));
if (lambda <= 0)
    Ktilde = Kbar + xi*abs(lambda)*eye(dim*n_node);
else
    Ktilde = Kbar;
end

end