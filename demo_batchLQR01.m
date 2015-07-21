function demo_batchLQR01
% Controller retrieval through a batch solution of linear quadratic optimal control (unconstrained linear MPC),
% by relying on a Gaussian mixture model (GMM) encoding of position and velocity data.
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
nbSamples = 3; %Number of demonstrations
nbRepros = 5; %Number of reproductions
nbData = 100; %Number of datapoints

model.nbStates = 5; %Number of Gaussians in the GMM
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


%% Load AMARSI handwriting data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('data/AMARSI/GShape.mat')
Data=[];
for n=1:nbSamples
	s(n).Data=[];
	for m=1:model.nbDeriv
		if m==1
			dTmp = spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),nbData)); %Resampling
		else
			dTmp = gradient(dTmp) / model.dt; %Compute derivatives
		end
		s(n).Data = [s(n).Data; dTmp];
	end
	Data = [Data s(n).Data]; 
end


%% Learning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%model = init_GMM_kmeans(Data,model);

%Initialization based on position data
model0 = init_GMM_kmeans(Data(1:model.nbVarPos,:), model);
[~, GAMMA2] = EM_GMM(Data(1:model.nbVarPos,:), model0);
model.Priors = model0.Priors;
for i=1:model.nbStates
	model.Mu(:,i) = Data * GAMMA2(i,:)';
	DataTmp = Data - repmat(model.Mu(:,i),1,nbData*nbSamples);
	model.Sigma(:,:,i) = DataTmp * diag(GAMMA2(i,:)) * DataTmp';
end

%Refinement of parameters
[model, H] = EM_GMM(Data, model);

%Set list of states according to first demonstration (alternatively, an HSMM can be used)
[~,qList] = max(H(:,1:nbData),[],1); %works also for nbStates=1

%Create single Gaussian N(MuQ,SigmaQ) based on optimal state sequence q, see Eq. (27)
MuQ = reshape(model.Mu(:,qList), model.nbVar*nbData, 1); 
SigmaQ = zeros(model.nbVar*nbData);
for t=1:nbData
	id = (t-1)*model.nbVar+1:t*model.nbVar;
	%MuQ(id,1) = model.Mu(:,qList(t));
	SigmaQ(id,id) = model.Sigma(:,:,qList(t));
end	
%SigmaQ can alternatively be computed with: 
%SigmaQ = (kron(ones(nbData,1), eye(model.nbVar)) * reshape(model.Sigma(:,:,qList), model.nbVar, model.nbVar*nbData)) .* kron(eye(nbData), ones(model.nbVar));


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

%Set matrices to compute the damped weighted least squares estimate, see Eq. (37)
SuInvSigmaQ = Su' / SigmaQ;
Rq = SuInvSigmaQ * Su + R;
%M = (SuInvSigmaQ * Su + R) \ SuInvSigmaQ; 
%M = inv(Su' / SigmaQ * Su + R) * Su' /SigmaQ;

%Reproductions
for n=1:nbRepros
	X = Data(:,1) + [randn(model.nbVarPos,1)*2E0; zeros(model.nbVarPos,1)];
 	rq = SuInvSigmaQ * (MuQ-Sx*X);
 	u = Rq \ rq; %Can also be computed with u = lscov(Rq, rq);
	%u = M * (MuQ-Sx*X);
	r(n).Data = reshape(Sx*X+Su*u, model.nbVar, nbData);
	
% 	%Simulate plant
% 	r(n).u = reshape(u, model.nbVarPos, nbData-1);
% 	for t=1:nbData-1
% 		%id = (t-1)*model.nbVar+1:t*model.nbVar;
% 		%r(n).u(:,t) = M(id,id) * (MuQ(id) - X(1:model.nbVar));
% 		X = A * X + B * r(n).u(:,t);
% 		r(n).Data(:,t) = X;
% 	end
end


%% Plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10 10 700 700],'color',[1 1 1]); hold on; axis off;
plotGMM(model.Mu(1:2,:), model.Sigma(1:2,1:2,:), [0.5 0.5 0.5]);
plot(Data(1,:), Data(2,:), 'k.');
for n=1:nbRepros
	plot(r(n).Data(1,:), r(n).Data(2,:), '-','linewidth',2,'color',[.8 0 0]);
	plot(r(n).Data(1,:), r(n).Data(2,:), '.','markersize',16,'color',[1 0 0]);
end
axis equal; 

%print('-dpng','graphs/demo_batchLQR01.png');
%pause;
%close all;
