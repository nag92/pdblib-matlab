function demo_iterativeLQR02
% Controller retrieval through an iterative solution of linear quadratic optimal control (finite horizon, 
% unconstrained linear MPC), by relying on a Gaussian mixture model (GMM) encoding of only position data
%
% Sylvain Calinon, Danilo Bruno, 2015
% http://programming-by-demonstration.org/lib/
%
% This source code is given for free! In exchange, we would be grateful if you cite
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
model.nbDeriv = 1; %Number of static & dynamic features (D=1 for [x1,x2])
model.nbVar = model.nbVarPos * model.nbDeriv; %Dimension of state vector
model.rfactor = 1E-6;	%Control cost in LQR
model.dt = 0.01; %Time step duration

%Dynamical System settings (discrete version)
A = kron([1, model.dt; 0, 1], eye(model.nbVarPos));
B = kron([0; model.dt], eye(model.nbVarPos));
%C = kron([1, 0], eye(model.nbVarPos));
%Control cost matrix
R = eye(model.nbVarPos) * model.rfactor;


%% Load Data
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
model = init_GMM_kmeans(Data,model);
%Refinement of parameters
[model, H] = EM_GMM(Data, model);
%Set list of states according to first demonstration (alternatively, an HSMM can be used)
[~,qList] = max(H(:,1:nbData),[],1); %works also for nbStates=1


%% Iterative LQR reproduction (finite horizon)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
P = zeros(model.nbVarPos*2,model.nbVarPos*2,nbData);
P(1:model.nbVarPos,1:model.nbVarPos,end) = inv(model.Sigma(:,:,qList(nbData)));
d = zeros(model.nbVarPos*2, nbData);
Q = zeros(model.nbVarPos*2);
for t=nbData-1:-1:1
	Q(1:model.nbVarPos,1:model.nbVarPos) = inv(model.Sigma(:,:,qList(t)));
	P(:,:,t) = Q - A' * (P(:,:,t+1) * B / (B' * P(:,:,t+1) * B + R) * B' * P(:,:,t+1) - P(:,:,t+1)) * A;
	d(:,t) = (A' - A'*P(:,:,t+1) * B / (R + B' * P(:,:,t+1) * B) * B' ) * (P(:,:,t+1) * ...
		( A * [model.Mu(:,qList(t)); zeros(model.nbVarPos,1)] - [model.Mu(:,qList(t+1)); zeros(model.nbVarPos,1)] ) + d(:,t+1));
end
%Reproduction with feedback (FB) and feedforward (FF) terms
for n=1:nbRepros
	X = [Data(:,1) + randn(model.nbVarPos,1)*2E0; zeros(model.nbVarPos,1)];
	r(n).X0 = X;
	for t=1:nbData
		r(n).Data(:,t) = X; %Log data
		K = (B' * P(:,:,t) * B + R) \ B' * P(:,:,t) * A; %FB term
		M = -(B' * P(:,:,t) * B + R) \ B' * (P(:,:,t) * ...
			(A * [model.Mu(:,qList(t)); zeros(model.nbVarPos,1)] - [model.Mu(:,qList(t)); zeros(model.nbVarPos,1)]) + d(:,t)); %FF term
		u = K * ([model.Mu(:,qList(t)); zeros(model.nbVarPos,1)] - X) + M; %Acceleration command with FB and FF terms
		X = A * X + B * u; %Update of state vector
	end
end


%% Plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10 10 1300 600],'color',[1 1 1]); 
%Plot position
subplot(1,2,1); hold on; 
plotGMM(model.Mu(1:2,:), model.Sigma(1:2,1:2,:), [0.5 0.5 0.5], .3);
for n=1:nbSamples
	plot(Data(1,(n-1)*nbData+1:n*nbData), Data(2,(n-1)*nbData+1:n*nbData), '-','color',[.7 .7 .7]);
end
for n=1:nbRepros
	plot(r(n).Data(1,:), r(n).Data(2,:), '-','linewidth',2,'color',[.8 0 0]); %Reproduction with iterative LQR
end
axis equal;
xlabel('x_1'); ylabel('x_2');

%Plot velocity
subplot(1,2,2); hold on; 
for n=1:nbSamples
	DataVel = gradient(Data(:,(n-1)*nbData+1:n*nbData)) / model.dt;
	plot(DataVel(1,:), DataVel(2,:), '-','color',[.7 .7 .7]);
end
for n=1:nbRepros
	plot(r(n).Data(3,:), r(n).Data(4,:), '-','linewidth',2,'color',[.8 0 0]); %Reproduction with iterative LQR
	plot(r(n).Data(3,1), r(n).Data(4,1), '.','markersize',18,'color',[.6 0 0]);
end
plot(0,0,'k+');
axis equal;
xlabel('dx_1'); ylabel('dx_2');

%print('-dpng','graphs/demo_iterativeLQR02.png');
pause;
close all;

