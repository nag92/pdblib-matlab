function demo_TPbatchLQR02
% Batch solution of a linear quadratic optimal control (unconstrained linear MPC) acting in multiple frames,
% which is equivalent to TP-GMM combined with LQR.
%
% Writing code takes time. Polishing it and making it available to others takes longer! 
% If some parts of the code were useful for your research of for a better understanding 
% of the algorithms, please reward the authors by citing the related publications, 
% and consider making your own research available in this way.
%
% @article{Calinon15,
%   author="Calinon, S.",
%   title="A Tutorial on Task-Parameterized Movement Learning and Retrieval",
%   journal="Intelligent Service Robotics",
%   year="2015"
% }
% 
% Copyright (c) 2015 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon, http://calinon.ch/
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model.nbStates = 3; %Number of Gaussians in the GMM
model.nbFrames = 2; %Number of candidate frames of reference
model.nbVarPos = 2; %Dimension of position data (here: x1,x2)
model.nbDeriv = 2; %Number of static & dynamic features (D=2 for [x,dx])
model.nbVar = model.nbVarPos * model.nbDeriv; %Dimension of state vector
model.rfactor = 1E-1;	%Control cost in LQR
model.dt = 0.01; %Time step duration
nbData = 200; %Number of datapoints in a trajectory
nbRepros = 5; %Number of reproductions

%Dynamical System settings (discrete version), see Eq. (33)
A = kron([1, model.dt; 0, 1], eye(model.nbVarPos));
B = kron([0; model.dt], eye(model.nbVarPos));
%Control cost matrix
R = eye(model.nbVarPos) * model.rfactor;
R = kron(eye(nbData-1),R);


%% Load 3rd order tensor data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Load 3rd order tensor data...');
% The MAT file contains a structure 's' with the multiple demonstrations. 's(n).Data' is a matrix data for
% sample n (with 's(n).nbData' datapoints). 's(n).p(m).b' and 's(n).p(m).A' contain the position and
% orientation of the m-th candidate coordinate system for this demonstration. 'Data' contains the observations
% in the different frames. It is a 3rd order tensor of dimension DC x P x N, with D=2 the dimension of a
% datapoint, C=2 the number of derivatives (incl. position), P=2 the number of candidate frames, and N=TM 
% the number of datapoints in a trajectory (T=200) multiplied by the number of demonstrations (M=5).
load('data/DataWithDeriv01.mat');


%% TP-GMM learning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('Parameters estimation of TP-GMM with EM...');
%model = init_tensorGMM_kmeans(Data, model); %Initialization

%Initialization based on position data
model0 = init_tensorGMM_kmeans(Data(1:model.nbVarPos,:,:), model);
[~,~,GAMMA2] = EM_tensorGMM(Data(1:model.nbVarPos,:,:), model0);
model.Priors = model0.Priors;
for i=1:model.nbStates
	for m=1:model.nbFrames
		DataTmp = squeeze(Data(:,m,:));
		model.Mu(:,m,i) = DataTmp * GAMMA2(i,:)';
		DataTmp = DataTmp - repmat(model.Mu(:,m,i),1,nbData*nbSamples);
		model.Sigma(:,:,m,i) = DataTmp * diag(GAMMA2(i,:)) * DataTmp';
	end
end

model = EM_tensorGMM(Data, model);


%% Reproduction with LQR for the task parameters used to train the model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Reproductions with batch LQR...');
%Construct Su and Sx matrices, see Eq. (35)
Su = zeros(model.nbVar*nbData, model.nbVarPos*(nbData-1));
Sx = kron(ones(nbData,1),eye(model.nbVar)); 
M = B;
for n=2:nbData
	%Build Sx matrix
	id1 = (n-1)*model.nbVar+1:nbData*model.nbVar;
	Sx(id1,:) = Sx(id1,:) * A;
	%Build Su matrix
	id1 = (n-1)*model.nbVar+1:n*model.nbVar; 
	id2 = 1:(n-1)*model.nbVarPos;
	Su(id1,id2) = M;
	M = [A*M(:,1:model.nbVarPos), M];
end

%Reproductions
for n=1:nbSamples
	
	%GMM projection, see Eq. (5)
	for i=1:model.nbStates
		for m=1:model.nbFrames
			s(n).p(m).Mu(:,i) = s(n).p(m).A * model.Mu(:,m,i) + s(n).p(m).b;
			s(n).p(m).Sigma(:,:,i) = s(n).p(m).A * model.Sigma(:,:,m,i) * s(n).p(m).A';
		end
	end
	
	%Compute best path for the n-th demonstration
	[~,s(n).q] = max(model.Pix(:,(n-1)*nbData+1:n*nbData),[],1); %works also for nbStates=1	
	
	%Build a reference trajectory for each frame, see Eq. (27)
	invSigmaQ = zeros(model.nbVar*nbData);
	for m=1:model.nbFrames
		s(n).p(m).MuQ = reshape(s(n).p(m).Mu(:,s(n).q), model.nbVar*nbData, 1);  
		s(n).p(m).SigmaQ = (kron(ones(nbData,1), eye(model.nbVar)) * reshape(s(n).p(m).Sigma(:,:,s(n).q), model.nbVar, model.nbVar*nbData)) ...
			.* kron(eye(nbData), ones(model.nbVar));
		invSigmaQ = invSigmaQ + inv(s(n).p(m).SigmaQ);
	end
	
	%Batch LQR (unconstrained linear MPC), see Eq. (37)
	SuInvSigmaQ = Su' * invSigmaQ;
	Rq = SuInvSigmaQ * Su + R;
	X = [s(1).Data0(:,1) + randn(model.nbVarPos,1)*0E0; zeros(model.nbVarPos,1)];
 	rq = zeros(model.nbVar*nbData,1);
	for m=1:model.nbFrames
		rq = rq + s(n).p(m).SigmaQ \ (s(n).p(m).MuQ - Sx*X);
	end
	rq = Su' * rq; 
 	u = Rq \ rq; %can also be computed with u = lscov(Rq, rq);
	r(n).Data = reshape(Sx*X+Su*u, model.nbVar, nbData);
end


%% Reproduction with LQR for new task parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('New reproductions with batch LQR...');
for n=1:nbRepros
	
	%Random generation of new task parameters
	for m=1:model.nbFrames
		id=ceil(rand(2,1)*nbSamples);
		w=rand(2); w=w/sum(w);
		rnew(n).p(m).b = s(id(1)).p(m).b * w(1) + s(id(2)).p(m).b * w(2);
		rnew(n).p(m).A = s(id(1)).p(m).A * w(1) + s(id(2)).p(m).A * w(2);
	end
	
	%GMM projection, see Eq. (5)
	for i=1:model.nbStates
		for m=1:model.nbFrames
			rnew(n).p(m).Mu(:,i) = rnew(n).p(m).A * model.Mu(:,m,i) + rnew(n).p(m).b;
			rnew(n).p(m).Sigma(:,:,i) = rnew(n).p(m).A * model.Sigma(:,:,m,i) * rnew(n).p(m).A';
		end
	end
	
	%Compute best path for the 1st demonstration (HSMM can alternatively be used here)
	[~,rnew(n).q] = max(model.Pix(:,1:nbData),[],1); %works also for nbStates=1
	
	%Build a reference trajectory for each frame, see Eq. (27)
	invSigmaQ = zeros(model.nbVar*nbData);
	for m=1:model.nbFrames
		rnew(n).p(m).MuQ = reshape(rnew(n).p(m).Mu(:,rnew(n).q), model.nbVar*nbData, 1);  
		rnew(n).p(m).SigmaQ = (kron(ones(nbData,1), eye(model.nbVar)) * ...
			reshape(rnew(n).p(m).Sigma(:,:,rnew(n).q), model.nbVar, model.nbVar*nbData)) .* kron(eye(nbData), ones(model.nbVar));
		invSigmaQ = invSigmaQ + inv(rnew(n).p(m).SigmaQ);
	end
	
	%Batch LQR (unconstrained linear MPC), see Eq. (37)
	SuInvSigmaQ = Su' * invSigmaQ;
	Rq = SuInvSigmaQ * Su + R;
	X = [s(1).Data0(:,1) + randn(model.nbVarPos,1)*0E0; zeros(model.nbVarPos,1)];
 	rq = zeros(model.nbVar*nbData,1);
	for m=1:model.nbFrames
		rq = rq + rnew(n).p(m).SigmaQ \ (rnew(n).p(m).MuQ - Sx*X);
	end
	rq = Su' * rq; 
 	u = Rq \ rq; %can also be computed with u = lscov(Rq, rq);
	rnew(n).Data = reshape(Sx*X+Su*u, model.nbVar, nbData);
end


%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[20,50,1300,500]);
xx = round(linspace(1,64,nbSamples));
clrmap = colormap('jet');
clrmap = min(clrmap(xx,:),.95);
limAxes = [-1.2 0.8 -1.1 0.9];
colPegs = [[.9,.5,.9];[.5,.9,.5]];

%DEMOS
subplot(1,3,1); hold on; box on; title('Demonstrations');
for n=1:nbSamples
	%Plot frames
	for m=1:model.nbFrames
		plot([s(n).p(m).b(1) s(n).p(m).b(1)+s(n).p(m).A(1,2)], [s(n).p(m).b(2) s(n).p(m).b(2)+s(n).p(m).A(2,2)], '-','linewidth',6,'color',colPegs(m,:));
		plot(s(n).p(m).b(1), s(n).p(m).b(2),'.','markersize',30,'color',colPegs(m,:)-[.05,.05,.05]);
	end
	%Plot trajectories
	plot(s(n).Data0(1,1), s(n).Data0(2,1),'.','markersize',12,'color',clrmap(n,:));
	plot(s(n).Data0(1,:), s(n).Data0(2,:),'-','linewidth',1.5,'color',clrmap(n,:));
end
axis(limAxes); axis square; set(gca,'xtick',[],'ytick',[]);

%REPROS
subplot(1,3,2); hold on; box on; title('Reproductions');
for n=1:nbSamples
	%Plot frames
	for m=1:model.nbFrames
		plot([s(n).p(m).b(1) s(n).p(m).b(1)+s(n).p(m).A(1,2)], [s(n).p(m).b(2) s(n).p(m).b(2)+s(n).p(m).A(2,2)], '-','linewidth',6,'color',colPegs(m,:));
		plot(s(n).p(m).b(1), s(n).p(m).b(2),'.','markersize',30,'color',colPegs(m,:)-[.05,.05,.05]);
	end
end
for n=1:nbSamples
	%Plot trajectories
	plot(r(n).Data(1,1), r(n).Data(2,1),'.','markersize',12,'color',clrmap(n,:));
	plot(r(n).Data(1,:), r(n).Data(2,:),'-','linewidth',1.5,'color',clrmap(n,:));
end
axis(limAxes); axis square; set(gca,'xtick',[],'ytick',[]);

%NEW REPROS
subplot(1,3,3); hold on; box on; title('New reproductions');
for n=1:nbRepros
	%Plot frames
	for m=1:model.nbFrames
		plot([rnew(n).p(m).b(1) rnew(n).p(m).b(1)+rnew(n).p(m).A(1,2)], [rnew(n).p(m).b(2) rnew(n).p(m).b(2)+rnew(n).p(m).A(2,2)], '-','linewidth',6,'color',colPegs(m,:));
		plot(rnew(n).p(m).b(1), rnew(n).p(m).b(2), '.','markersize',30,'color',colPegs(m,:)-[.05,.05,.05]);
	end
end
for n=1:nbRepros
	%Plot trajectories
	plot(rnew(n).Data(1,1), rnew(n).Data(2,1),'.','markersize',12,'color',[.2 .2 .2]);
	plot(rnew(n).Data(1,:), rnew(n).Data(2,:),'-','linewidth',1.5,'color',[.2 .2 .2]);
end
axis(limAxes); axis square; set(gca,'xtick',[],'ytick',[]);

%print('-dpng','graphs/demo_TPbatchLQR02.png');
%pause;
%close all;
