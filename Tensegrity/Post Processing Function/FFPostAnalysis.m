function Result = FFPostAnalysis(Tensegrity)

% tolerance for calculation the rank of a matrix.
tolerance = 1.0e-6;% a value close to zero.
d = Tensegrity.dim;
nE = Tensegrity.n_elem;
nN = Tensegrity.n_node;
% Maxwell number
Mx = nE - (d*nN - d*(d+1)/2);
[A,D] = AssemblyAD(Tensegrity);
rA = numel(find(svd(A)>tolerance));%rank(A)
rD = numel(find(svd(D)>tolerance));%rank(D)
nS = nE - rA;
nM = d*nN - d*(d+1)/2 - rA;
nD = Tensegrity.n_node - rD;% nD >= d+1
sigma = find(eig(D)<-1.0e-8);% eliminate the influence of the calculation error.

% check equilibrium equations.
dim = Tensegrity.dim;n_node = Tensegrity.n_node;
q = zeros(dim*n_node,1);% node coordinates, row vector.
for i = 1:n_node
    q(dim*i-(dim-1):dim*i) = Tensegrity.Node(i,1:dim);
end
Dx = D*q(1:3:end);
Dy = D*q(2:3:end);
Dz = D*q(3:3:end);
U = GetEnergy(Tensegrity, q);

[L, DirV, Fint] = GetState(Tensegrity);
n_elem = Tensegrity.n_elem;s = zeros(n_elem,1);
for i = 1:n_elem
    s(i) = Fint(i)/L(i);
end
As = A*s;

svdA = svd(A);
svdD = svd(D);
eigD = eig(D);

% tangent stiffness matrix.
[K] = Assembly(Tensegrity);
K(9,:) = [];K(:,9) = [];%[1,2,3,5,6,9]
K(6,:) = [];K(:,6) = [];
K(5,:) = [];K(:,5) = [];
K(1:3,:) = [];K(:,1:3) = [];
eigK = eig(K);
svdK = svd(K);

% pre-strain.
prestrain = zeros(nE,1);
for i = 1:nE
    CL = norm(Tensegrity.Node(Tensegrity.Elem(i,1),:)-Tensegrity.Node(Tensegrity.Elem(i,2),:));
    prestrain(i) = (CL-Tensegrity.L0(i))/Tensegrity.L0(i);
end

Result.Mx = Mx;
Result.U = U;
Result.A = A;
Result.D = D;
Result.rA = rA;
Result.rD = rD;
Result.nE = nE;
Result.nN = nN;
Result.nS = nS;
Result.nM = nM;
Result.nD = nD;
Result.sigma = sigma;
Result.Dx = Dx;
Result.Dy = Dy;
Result.Dz = Dz;
Result.As = As;
Result.s = s/abs(s(1));% normalization.
Result.isZero = find(s==0);
Result.svdA = svdA;
Result.svdD = svdD;
Result.eigD = eigD;
Result.K = K;
Result.svdK = svdK;
Result.eigK = eigK;
Result.prestrain = prestrain;

% evaluation results.
if (Mx > 0)
    fprintf('Maxwell number = %d, tensegrity is statically indeterminate.\n',Mx);
elseif (Mx == 0)
    fprintf('Maxwell number = %d, tensegrity is statically determinate.\n',Mx);
elseif (Mx < 0)
    fprintf('Maxwell number = %d, tensegrity is kinematically indeterminate mechanism.\n',Mx);
end

end