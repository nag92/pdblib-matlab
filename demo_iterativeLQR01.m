function demo_iterativeLQR01
% Controller retrieval through an iterative solution of linear quadratic optimal control (finite horizon, 
% unconstrained linear MPC), by relying on a Gaussian mixture model (GMM) encoding of position and velocity data, 
% including comparison with batch LQR.
%
% Writing code takes time. Polishing it and making it available to others takes longer! 
% If some parts of the code were useful for your research of for a better understanding 
% of the algorithms, please reward the authors by citing the related publications, 
% and consider making your own research available in this way.
%
% @article{Calinon16JIST,
%   author="Calinon, S.",
%   title="A Tutorial on Task-Parameterized Movement Learning and Retrieval",
%   journal="Intelligent Service Robotics",
%		publisher="Springer Berlin Heidelberg",
%		doi="10.1007/s11370-015-0187-9",
%		year="2016",
%		volume="9",
%		number="1",
%		pages="1--29"
% }
%
% @inproceedings{Calinon14,
%   author="Calinon, S. and Bruno, D. and Caldwell, D. G.",
%   title="A task-parameterized probabilistic model with minimal intervention control",
%   booktitle="Proc. {IEEE} Intl Conf. on Robotics and Automation ({ICRA})",
%   year="2014",
%   month="May-June",
%   address="Hong Kong, China",
%   pages="3339--3344"
% }
% 
% Copyright (c) 2015 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon (http://calinon.ch/) and Danilo Bruno (danilo.bruno@iit.it)
% 
% This file is part of PbDlib, http://www.idiap.ch/software/pbdlib/
% 
% PbDlib is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License version 3 as
% published by the Free Software Foundation.
% 
% PbDlib is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with PbDlib. If not, see <http://www.gnu.org/licenses/>.

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

%Dynamical System settings (discrete version)
A = kron([1, model.dt; 0, 1], eye(model.nbVarPos));
B = kron([0; model.dt], eye(model.nbVarPos));
%Control cost matrix
R = eye(model.nbVarPos) * model.rfactor;


%% Load handwriting data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('data/2Dletters/G.mat');
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


%% Iterative LQR reproduction (finite horizon)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
P = zeros(model.nbVar,model.nbVar,nbData);
P(:,:,end) = inv(model.Sigma(:,:,qList(nbData)));
d = zeros(model.nbVar, nbData);
for t=nbData-1:-1:1
	Q = inv(model.Sigma(:,:,qList(t)));
	P(:,:,t) = Q - A' * (P(:,:,t+1) * B / (B' * P(:,:,t+1) * B + R) * B' * P(:,:,t+1) - P(:,:,t+1)) * A;
	d(:,t) = (A' - A'*P(:,:,t+1) * B / (R + B' * P(:,:,t+1) * B) * B' ) * (P(:,:,t+1) * (A * model.Mu(:,qList(t)) - model.Mu(:,qList(t+1))) + d(:,t+1));
end
%Reproduction with feedback (FB) and feedforward (FF) terms
for n=1:nbRepros
	X = Data(:,1) + [randn(model.nbVarPos,1)*2E0; zeros(model.nbVarPos,1)];
	r(n).X0 = X;
	for t=1:nbData
		r(n).Data(:,t) = X; %Log data
		K = (B' * P(:,:,t) * B + R) \ B' * P(:,:,t) * A; %FB term
		M = -(B' * P(:,:,t) * B + R) \ B' * (P(:,:,t) * (A * model.Mu(:,qList(t)) - model.Mu(:,qList(t))) + d(:,t)); %FF term
		u = K * (model.Mu(:,qList(t)) - X) + M; %Acceleration command with FB and FF terms
		X = A * X + B * u; %Update of state vector
	end
end


%% Batch LQR reproduction (for comparison)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Create single Gaussian N(MuQ,SigmaQ) based on optimal state sequence q, see Eq. (27)
MuQ = reshape(model.Mu(:,qList), model.nbVar*nbData, 1); 
SigmaQ = (kron(ones(nbData,1), eye(model.nbVar)) * reshape(model.Sigma(:,:,qList), model.nbVar, model.nbVar*nbData)) .* kron(eye(nbData), ones(model.nbVar));
%Build Sx and Su matrices for batch LQR, see Eq. (35)
Su = zeros(model.nbVar*nbData, model.nbVarPos*(nbData-1));
Sx = kron(ones(nbData,1),eye(model.nbVar)); 
M = B;
for n=2:nbData
	id1 = (n-1)*model.nbVar+1:nbData*model.nbVar;
	Sx(id1,:) = Sx(id1,:) * A;
	id1 = (n-1)*model.nbVar+1:n*model.nbVar; 
	id2 = 1:(n-1)*model.nbVarPos;
	Su(id1,id2) = M;
	M = [A*M(:,1:model.nbVarPos), M];
end
%Set matrices to compute the damped weighted least squares estimate, see Eq. (37)
SuInvSigmaQ = Su' / SigmaQ;
Rq = SuInvSigmaQ * Su + kron(eye(nbData-1),R);
%Reproductions
for n=1:nbRepros
	X = r(n).X0;
 	rq = SuInvSigmaQ * (MuQ-Sx*X);
 	u = Rq \ rq; 
	r2(n).Data = reshape(Sx*X+Su*u, model.nbVar, nbData);
end


%% Plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10 10 1300 500],'color',[1 1 1]); 
%Plot position
subplot(1,2,1); hold on; 
plotGMM(model.Mu(1:2,:), model.Sigma(1:2,1:2,:), [0.5 0.5 0.5], .3);
for n=1:nbSamples
	plot(Data(1,(n-1)*nbData+1:n*nbData), Data(2,(n-1)*nbData+1:n*nbData), '-','color',[.7 .7 .7]);
end
for n=1:nbRepros
	h(1) = plot(r(n).Data(1,:), r(n).Data(2,:), '-','linewidth',2,'color',[.8 0 0]); %Reproduction with iterative LQR
	h(2) = plot(r2(n).Data(1,:), r2(n).Data(2,:), '--','linewidth',2,'color',[0 .8 0]); %Reproduction with batch LQR
end
axis equal; 
xlabel('x_1'); ylabel('x_2');
legend(h,'Iterative LQR','Batch LQR');

%Plot velocity
subplot(1,2,2); hold on; 
plotGMM(model.Mu(3:4,:), model.Sigma(3:4,3:4,:), [0.5 0.5 0.5], .3);
for n=1:nbSamples
	plot(Data(3,(n-1)*nbData+1:n*nbData), Data(4,(n-1)*nbData+1:n*nbData), '-','color',[.7 .7 .7]);
end
for n=1:nbRepros
	plot(r(n).Data(3,:), r(n).Data(4,:), '-','linewidth',2,'color',[.8 0 0]); %Reproduction with iterative LQR
	plot(r(n).Data(3,1), r(n).Data(4,1), '.','markersize',18,'color',[.6 0 0]);
	plot(r2(n).Data(3,:), r2(n).Data(4,:), '--','linewidth',2,'color',[0 .8 0]); %Reproduction with batch LQR
end
plot(0,0,'k+');
axis equal;
xlabel('dx_1'); ylabel('dx_2');


%print('-dpng','graphs/demo_iterativeLQR01.png');
%pause;
%close all;

