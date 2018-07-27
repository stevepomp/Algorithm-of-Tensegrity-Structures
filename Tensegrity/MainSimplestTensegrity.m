% % % % % % % % % % % % % % % % % % % % 
clear all;close all;clc;colordef white;
addpath(genpath(pwd));tic;

% Tensegrity = GetOctahedron();
% Tensegrity = GetOctahedronVariation(0.3);

L = 2.5; alpha = 0.25; beta = 0.5; gamma = 2/(sqrt(5)+2);
Tensegrity = GetSimplestTensegrity(L, alpha, beta, gamma);

% Tensegrity.LabelOn = 0;
% Tensegrity = FormFinding(Tensegrity);
% % PlotTensegrity(Tensegrity);
% FFResult = FFPostAnalysis(Tensegrity);

% % % % % % % % % % % % % % % % % % % % 
% constraint.
DOF_Active = ones(Tensegrity.n_node,3);
DOF_Active(1,:) = [0,0,0];
DOF_Active(2,:) = [0,0,0];
DOF_Active(3,:) = [0,0,0];
DOF_Active(4,:) = [1,1,0];
Model = Tensegrity2Model(Tensegrity, DOF_Active);

% reference load.
Model.Load = zeros(Model.n_node,3);
Model.Load(4,:) = [1e3, 0, 0];

% control parameters for solving.
Model.Control.lambda = 0;
Model.Control.lambdamin = -100;
Model.Control.lambdamax = 1322;
Model.Control.Dlambda = 1.0;
Model.Control.nLoadStep = 1000;
Model.Control.nIteration = 200;
Model.Control.Nreference = 6;% fixed.
Model.Control.disp_tolerance = 1.0e-10;
Model.Control.force_tolerance = 1.0e-10;

NSResult = NonlinearStaticsSolver(Model);

iNSResult = NSResult(end);
iNSResult.LabelOn = 0;
iNSResult.radius = 0.02;
PlotNSResult(iNSResult,Model);


npt = numel(NSResult);
Force = zeros(npt,1);
P4X = zeros(npt,1);
P4Y = zeros(npt,1);
for i = 1:npt
    Force(i) = NSResult(i).lambda;
    P4X(i) = NSResult(i).node(4,1);
    P4Y(i) = NSResult(i).node(4,2);
end

figure; hold on;
plot(Force, P4X, 'r-x');
plot(Force, P4Y, 'b-x');

x_truss = load('Data/Simplest Tensegrity/x_truss');
y_truss = load('Data/Simplest Tensegrity/y_truss');
f_truss = load('Data/Simplest Tensegrity/f_truss');
plot(f_truss, x_truss, 'r-', 'LineWidth', 2);
plot(f_truss, y_truss, 'b-', 'LineWidth', 2);

x_exact = load('Data/Simplest Tensegrity/x_exact');
y_exact = load('Data/Simplest Tensegrity/y_exact');
f_exact = load('Data/Simplest Tensegrity/f_exact');
plot(f_exact, x_exact, 'g-', 'LineWidth', 2);
plot(f_exact, y_exact, 'y-', 'LineWidth', 2);

xlabel('Force (kN)');ylabel('Position (m)');
legend('P4X', 'P4Y', 'P4Xtruss', 'P4Ytruss', 'P4Xexact', 'P4Yexact');

% % % % % % % % % % % % % % % % % % % % 
toc;
% % % % % % % % % % % % % % % % % % % % 
