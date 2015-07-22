function demo_testLQR04
% Demonstration of the coordination capability of linear quadratic optimal control 
% (unconstrained linear MPC) combined with full precision matrices.
%
% Sylvain Calinon, 2015
% http://programming-by-demonstration.org/lib/
%
% This source code is given for free! In exchange, I would be grateful if you cite
% the following reference in any academic publication that uses this code or part of it:
%
% @article{Calinon15,
%   author="Calinon, S.",
%   title="A tutorial on task-parameterized movement learning and retrieval",
%   year="2015",
% }

addpath('./m_fcts/');


%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nbRepros = 5; %Number of reproductions
nbData = 100; %Number of datapoints

model.nbVarPos = 2; %Dimension of position data (here: x1,x2)
model.nbDeriv = 2; %Number of static & dynamic features (D=2 for [x,dx])
model.nbVar = model.nbVarPos * model.nbDeriv; %Dimension of state vector
model.rfactor = 1E-6;	%Control cost in LQR
model.dt = 0.01; %Time step duration

%Dynamical System settings (discrete version), see Eq. (33)
A = kron([1, model.dt; 0, 1], eye(model.nbVarPos));
B = kron([0; model.dt], eye(model.nbVarPos));
C = kron([1, 0], eye(model.nbVarPos));
%Control cost matrix
R = eye(model.nbVarPos) * model.rfactor;
R = kron(eye(nbData-1),R);


%% Setting of coordination constraint
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Create single Gaussian N(MuQ,SigmaQ), see Eq. (27)
MuQ = zeros(model.nbVar*nbData, 1);
%Diagonal covariance matrices (-> without coordination)
Sigma0 = eye(model.nbVar)*1E1;
Sigma0(1:model.nbVarPos,1:model.nbVarPos) = eye(model.nbVarPos)*1E-1;
SigmaQ0 = kron(eye(nbData),Sigma0);
%Full covariance matrices (-> with coordination)
V = [.1; .5];
Sigma = eye(model.nbVar)*1E1;
Sigma(1:model.nbVarPos,1:model.nbVarPos) = V*V' + eye(model.nbVarPos)*1E-2;
SigmaQ = kron(eye(nbData),Sigma);


%% Batch LQR reproduction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Su = zeros(model.nbVar*nbData, model.nbVarPos*(nbData-1));
Sx = kron(ones(nbData,1),eye(model.nbVar)); 
M = B;
for n=2:nbData
	%Build Sx matrix, see Eq. (35)
	id1 = (n-1)*model.nbVar+1:nbData*model.nbVar;
	Sx(id1,:) = Sx(id1,:) * A;
	%Build Su matrix, see Eq. (35)
	id1 = (n-1)*model.nbVar+1:n*model.nbVar; 
	id2 = 1:(n-1)*model.nbVarPos;
	Su(id1,id2) = M;
	M = [A*M(:,1:model.nbVarPos), M];
end

for n=1:nbRepros
	X = [1; 1; 0; 0] + [randn(2,1)*2E-1; zeros(2,1)];
	%Reproductions without coordination
 	u = (Su'/SigmaQ0*Su + R) \ (Su'/SigmaQ0 * (MuQ-Sx*X)); 
	r0(n).Data = reshape(Sx*X+Su*u, model.nbVar, nbData);
	%Reproductions with coordination
	u = (Su'/SigmaQ*Su + R) \ (Su'/SigmaQ * (MuQ-Sx*X)); 
	r(n).Data = reshape(Sx*X+Su*u, model.nbVar, nbData);
end


%% Plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10 10 700 700],'color',[1 1 1]); hold on; axis off; 
plotGMM(zeros(2,1), Sigma0(1:2,1:2), [.8 0 0], .3);
plotGMM(zeros(2,1), Sigma(1:2,1:2), [0 .8 0], .3);
for n=1:nbRepros
	%Plot reproductions without coordination
	plot(r0(n).Data(1,:), r0(n).Data(2,:), '-','linewidth',2,'color',[.8 0 0]);
	plot(r0(n).Data(1,1:2:end), r0(n).Data(2,1:2:end), '.','markersize',16,'color',[1 0 0]);
	%Plot reproductions with coordination
	plot(r(n).Data(1,:), r(n).Data(2,:), '-','linewidth',2,'color',[0 .8 0]);
	plot(r(n).Data(1,1:2:end), r(n).Data(2,1:2:end), '.','markersize',16,'color',[0 1 0]);
end
axis equal; 

%print('-dpng','graphs/demo_testLQR04.png');
%pause;
%close all;

