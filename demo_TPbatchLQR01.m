function demo_TPbatchLQR01
% Task-parameterized GMM encoding position and velocity data, combined with a batch solution 
% of linear quadratic optimal control (unconstrained linear MPC).
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
model.nbStates = 6; %Number of Gaussians in the GMM
model.nbFrames = 2; %Number of candidate frames of reference
model.nbVarPos = 2; %Dimension of position data (here: x1,x2)
model.nbDeriv = 2; %Number of static & dynamic features (D=2 for [x,dx])
model.nbVar = model.nbVarPos * model.nbDeriv; %Dimension of state vector
model.rfactor = 1E-1;	%Control cost in LQR
model.dt = 0.01; %Time step duration
nbData = 200; %Number of datapoints in a trajectory
nbRepros = 5; %Number of reproductions

%Control cost matrix
R = eye(model.nbVarPos) * model.rfactor;
R = kron(eye(nbData-1),R);


%% Dynamical System settings (discrete version)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %Integration with Euler method 
% Ac1d = diag(ones(model.nbDeriv-1,1),1); 
% Bc1d = [zeros(model.nbDeriv-1,1); 1];
% A = kron(eye(model.nbDeriv)+Ac1d*model.dt, eye(model.nbVarPos)); 
% B = kron(Bc1d*model.dt, eye(model.nbVarPos));

%Integration with higher order Taylor series expansion
A1d = zeros(model.nbDeriv);
for i=0:model.nbDeriv-1
	A1d = A1d + diag(ones(model.nbDeriv-i,1),i) * model.dt^i * 1/factorial(i); %Discrete 1D
end
B1d = zeros(model.nbDeriv,1); 
for i=1:model.nbDeriv
	B1d(model.nbDeriv-i+1) = model.dt^i * 1/factorial(i); %Discrete 1D
end
A = kron(A1d, eye(model.nbVarPos)); %Discrete nD
B = kron(B1d, eye(model.nbVarPos)); %Discrete nD

% %Conversion with control toolbox
% Ac1d = diag(ones(model.nbDeriv-1,1),1); %Continuous 1D
% Bc1d = [zeros(model.nbDeriv-1,1); 1]; %Continuous 1D
% Cc1d = [1, zeros(1,model.nbDeriv-1)]; %Continuous 1D
% sysd = c2d(ss(Ac1d,Bc1d,Cc1d,0), model.dt);
% A = kron(sysd.a, eye(model.nbVarPos)); %Discrete nD
% B = kron(sysd.b, eye(model.nbVarPos)); %Discrete nD


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
model = init_tensorGMM_kbins(s, model);

% %Initialization based on position data
% model0 = init_tensorGMM_kmeans(Data(1:model.nbVarPos,:,:), model);
% [~,~,GAMMA2] = EM_tensorGMM(Data(1:model.nbVarPos,:,:), model0);
% model.Priors = model0.Priors;
% for i=1:model.nbStates
% 	for m=1:model.nbFrames
% 		DataTmp = squeeze(Data(:,m,:));
% 		model.Mu(:,m,i) = DataTmp * GAMMA2(i,:)';
% 		DataTmp = DataTmp - repmat(model.Mu(:,m,i),1,nbData*nbSamples);
% 		model.Sigma(:,:,m,i) = DataTmp * diag(GAMMA2(i,:)) * DataTmp';
% 	end
% end
% [~, GAMMA] = computeGamma(Data, model); %See 'computeGamma' function below
% model.Pix = GAMMA ./ repmat(sum(GAMMA,2),1,nbData*nbSamples);
	
model = EM_tensorGMM(Data, model);


%% Reproduction with LQR for the task parameters used to train the model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Reproductions with batch LQR...');
for n=1:nbSamples
	%Reconstruct GMM 
	[s(n).Mu, s(n).Sigma] = productTPGMM0(model, s(n).p);
	%Compute best path for the n-th demonstration
	[~,s(n).q] = max(model.Pix(:,(n-1)*nbData+1:n*nbData),[],1); %works also for nbStates=1	
	%Build stepwise reference trajectory, see Eq. (27)
	MuQ = reshape(s(n).Mu(:,s(n).q), model.nbVar*nbData, 1); 
	SigmaQ = (kron(ones(nbData,1), eye(model.nbVar)) * reshape(s(n).Sigma(:,:,s(n).q), model.nbVar, model.nbVar*nbData)) .* kron(eye(nbData), ones(model.nbVar));
	%Batch LQR (unconstrained linear MPC), see Eq. (37)
	SuInvSigmaQ = Su' / SigmaQ;
	Rq = SuInvSigmaQ * Su + R;
	X = [s(1).Data0(:,1) + randn(model.nbVarPos,1)*0E0; zeros(model.nbVarPos,1)];
 	rq = SuInvSigmaQ * (MuQ-Sx*X);
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
	%Reconstruct GMM 
	[rnew(n).Mu, rnew(n).Sigma] = productTPGMM0(model, rnew(n).p);
	%Compute best path for the 1st demonstration (HSMM can alternatively be used here)
	[~,rnew(n).q] = max(model.Pix(:,1:nbData),[],1); %works also for nbStates=1	
	%Build stepwise reference trajectory, see Eq. (27)
	MuQ = reshape(rnew(n).Mu(:,rnew(n).q), model.nbVar*nbData, 1); 
	SigmaQ = (kron(ones(nbData,1), eye(model.nbVar)) * reshape(rnew(n).Sigma(:,:,rnew(n).q), model.nbVar, model.nbVar*nbData)) .* kron(eye(nbData), ones(model.nbVar));
	%Batch LQR (unconstrained linear MPC), see Eq. (37)
	SuInvSigmaQ = Su' / SigmaQ;
	Rq = SuInvSigmaQ * Su + R;
	X = [s(1).Data0(:,1) + randn(model.nbVarPos,1)*0E0; zeros(model.nbVarPos,1)];
 	rq = SuInvSigmaQ * (MuQ-Sx*X);
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
for n=1:nbSamples
	%Plot Gaussians
	plotGMM(s(n).Mu(1:2,:), s(n).Sigma(1:2,1:2,:), [.5 .5 .5], .4);
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
for n=1:nbRepros
	%Plot Gaussians
	plotGMM(rnew(n).Mu(1:2,:), rnew(n).Sigma(1:2,1:2,:), [.5 .5 .5], .4);
end
axis(limAxes); axis square; set(gca,'xtick',[],'ytick',[]);

%Plot position
figure; 
subplot(2,1,1); hold on;
m=1;
for n=1:1 %nbSamples
	msh=[]; x0=[];
	for t=1:nbData-1
		if size(msh,2)==0
			msh(:,1) = [t; s(n).Mu(m,s(n).q(t))];
		end
		if t==nbData-1 || s(n).q(t+1)~=s(n).q(t)
			msh(:,2) = [t+1; s(n).Mu(m,s(n).q(t))];
			sTmp = s(n).Sigma(m,m,s(n).q(t))^.5;
			msh2 = [msh(:,1)+[0;sTmp], msh(:,2)+[0;sTmp], msh(:,2)-[0;sTmp], msh(:,1)-[0;sTmp], msh(:,1)+[0;sTmp]];
			patch(msh2(1,:), msh2(2,:), [.85 .85 .85],'edgecolor',[.7 .7 .7]);
			plot(msh(1,:), msh(2,:), '-','linewidth',3,'color',[.7 .7 .7]);
			x0 = [x0 msh];
			msh=[];
		end
	end
	plot(s(n).Data0(m,:),':');
	plot(r(n).Data(m,:),'-');
end
xlabel('t'); ylabel('x_1');

%Plot velocity
subplot(2,1,2); hold on;
m=3;
for n=1:1 %nbSamples
	msh=[]; x0=[];
	for t=1:nbData-1
		if size(msh,2)==0
			msh(:,1) = [t; s(n).Mu(m,s(n).q(t))];
		end
		if t==nbData-1 || s(n).q(t+1)~=s(n).q(t)
			msh(:,2) = [t+1; s(n).Mu(m,s(n).q(t))];
			sTmp = s(n).Sigma(m,m,s(n).q(t))^.5;
			msh2 = [msh(:,1)+[0;sTmp], msh(:,2)+[0;sTmp], msh(:,2)-[0;sTmp], msh(:,1)-[0;sTmp], msh(:,1)+[0;sTmp]];
			patch(msh2(1,:), msh2(2,:), [.85 .85 .85],'edgecolor',[.7 .7 .7]);
			plot(msh(1,:), msh(2,:), '-','linewidth',3,'color',[.7 .7 .7]);
			x0 = [x0 msh];
			msh=[];
		end
	end
	%plot(s(n).Data0(m,:),':');
	plot(r(n).Data(m,:),'-');
end
xlabel('t'); ylabel('dx_1');


%print('-dpng','graphs/demo_TPbatchLQR01.png');
%pause;
%close all;

