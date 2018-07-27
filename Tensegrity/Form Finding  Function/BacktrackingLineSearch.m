function [alpha] = BacktrackingLineSearch(Tensegrity, q0, Dq, m)
% Backtracking Line search algorithm.

if (m < eps)
    alpha = 1;return;
end

U0 = GetEnergy(Tensegrity, q0);

rho = 0.5;tau = 0.5;t = -rho*m;
alpha = 1;

while (1)
    qnew = q0 + alpha*Dq;
    Unew = GetEnergy(Tensegrity, qnew);
    if (U0 - Unew >= alpha*t  || alpha < 1.0e-6)
        break;
    else
        alpha = tau*alpha;
    end
end

end