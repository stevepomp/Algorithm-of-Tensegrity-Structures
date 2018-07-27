% % % % % % % % % % % % % % % % % % % % 
clear all;close all;clc;colordef white;
addpath(genpath(pwd));tic;

% Tensegrity = GetOctahedron();
 Tensegrity = GetOctahedronVariation(0.3);

% v = 3; j = 1; lc1 = 1.0; lc2 = 1.0; lc3 = 1.5; lb = 2.0;%
% Tensegrity = GetPrismaticTensegrity(v, j, lc1, lc2, lc3, lb);

% v = 3; j = 1; lc1 = 1.0; lc2 = 1.0; lc3 = 1.5; lc4 = 1.85; lb = 2.0;%
% Tensegrity = GetAntiPrismaticTensegrity(v, j, lc1, lc2, lc3, lc4, lb);

% v = 3; j = 1; lc1 = 1.0; lc2 = 1.0; lc3 = 1.5; lb = 2.0;%
% Tensegrity = GetPrismaticVariation(v, j, lc1, lc2, lc3, lb);

% Tensegrity = GetRegularTruncatedDodecahedral(1, 0.36);
% Tensegrity = GetRegularTruncatedIcosahedral(1, 0.36);

% Tensegrity = GetStellaOctangula();

Tensegrity.LabelOn = 0;
Tensegrity = AffineTransform3D(Tensegrity);% for 3D tensegrity
Tensegrity = FormFinding(Tensegrity);

PlotTensegrity(Tensegrity);
% PlotOctahedral(Tensegrity);
% PlotOctahedralVariation(Tensegrity);
% PlotPrismatic(Tensegrity);
% PlotTruncatedTensegrity(Tensegrity);

FFResult = FFPostAnalysis(Tensegrity);

% % % % % % % % % % % % % % % % % % % % 
% constraint.
DOF_Active = ones(Tensegrity.n_node,3);
DOF_Active(1,:) = [0,0,0];
DOF_Active(2,:) = [1,0,0];
DOF_Active(3,:) = [1,1,0];

Model = Tensegrity2Model(Tensegrity, DOF_Active);

% reference load.
Model.Load = zeros(Model.n_node,3);
Model.Load(10,:) = [0, 0, -10];
Model.Load(11,:) = [0, 0, -10];
Model.Load(12,:) = [0, 0, -10];

% control parameters for solving.
Model.Control.lambda = 0;
Model.Control.lambdamin = -100;
Model.Control.lambdamax = 100;
Model.Control.Dlambda = 1.0;
Model.Control.nLoadStep = 1000;
Model.Control.nIteration = 200;
Model.Control.Nreference = 6;% fixed.
Model.Control.disp_tolerance = 1.0e-10;
Model.Control.force_tolerance = 1.0e-10;

NSResult = NonlinearStaticsSolver(Model);

iNSResult = NSResult(end);
iNSResult.LabelOn = 0;
iNSResult.radius = 0.01;
PlotNSResult(iNSResult, Model);
% % % PlotNSResultWithField(iNSResult, Model, iNSResult.stress);

% % % % % % % % % % % % % % % % % % % % 
toc;
% % % % % % % % % % % % % % % % % % % % 
