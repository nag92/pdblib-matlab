function demo_trajTPGMM01
%Example of TP-GMM with trajectory GMM encoding
%Sylvain Calinon, 2015

addpath('./m_fcts/');

%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model.nbStates = 3; %Number of Gaussians in the GMM
model.nbFrames = 2; %Number of candidate frames of reference
model.nbVarPos = 2; %Dimension of position data (here: x1,x2)
model.nbDeriv = 3; %Number of static&dynamic features (D=2 for [x,dx], D=3 for [x,dx,ddx], etc.)
model.dt = 0.01; %Time step
nbRepros = 8; %Number of reproductions with new situations randomly generated

%% Load 3rd order tensor data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Load 3rd order tensor data...');
% The MAT file contains a structure 's' with the multiple demonstrations. 's(n).Data' is a matrix data for
% sample n (with 's(n).nbData' datapoints). 's(n).p(m).b' and 's(n).p(m).A' contain the position and
% orientation of the m-th candidate coordinate system for this demonstration. 'Data' contains the observations
% in the different frames. It is a 3rd order tensor of dimension D x P x N, with D=3 the dimension of a
% datapoint, P=2 the number of candidate frames, and N=200x4 the number of datapoints in a trajectory
% multiplied by the number of demonstrations.
load('data/DataLQR01.mat');

%Compute derivatives
nbD = s(1).nbData;
%Create transformation matrix to compute [X; DX; DDX]
D = (diag(ones(1,nbD-1),-1)-eye(nbD)) / model.dt;
D(end,end) = 0;
%Create 3rd order tensor data with XHAT instead of X
model.nbVar = model.nbVarPos * model.nbDeriv;
Data = zeros(model.nbVar, model.nbFrames, nbD);
for n=1:nbSamples
	DataTmp = s(n).Data0(2:end,:);
	for k=1:model.nbDeriv-1
		DataTmp = [DataTmp; s(n).Data0(2:end,:)*D^k]; %Compute derivatives
	end
	for m=1:model.nbFrames
		s(n).p(m).b = [s(n).p(m).b(2:end); zeros((model.nbDeriv-1)*model.nbVarPos,1)];
		s(n).p(m).A = kron(eye(model.nbDeriv), s(n).p(m).A(2:end,2:end));
		Data(:,m,(n-1)*nbD+1:n*nbD) = s(n).p(m).A \ (DataTmp - repmat(s(n).p(m).b, 1, nbD));
	end
end

%Construct PHI operator (big sparse matrix)
[PHI,PHI1,T,T1] = constructPHI(model,nbD,nbSamples); 


%% TP-GMM learning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('Parameters estimation of TP-GMM with EM:');
%model = init_tensorGMM_timeBased(Data, model); %Initialization
model = init_tensorGMM_kmeans(Data, model); %Initialization
model = EM_tensorGMM(Data, model);


%% Reproduction for the task parameters used to train the model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Reproductions...');
%DataIn = [1:s(1).nbData] * model.dt;
for n=1:nbSamples
	%Products of linearly transformed Gaussians
	for i=1:model.nbStates
		SigmaTmp = zeros(model.nbVar);
		MuTmp = zeros(model.nbVar,1);
		for m=1:model.nbFrames
			MuP = s(n).p(m).A * model.Mu(:,m,i) + s(n).p(m).b;
			SigmaP = s(n).p(m).A * model.Sigma(:,:,m,i) * s(n).p(m).A';
			SigmaTmp = SigmaTmp + inv(SigmaP);
			MuTmp = MuTmp + SigmaP\MuP;
		end
		r(n).Sigma(:,:,i) = inv(SigmaTmp);
		r(n).Mu(:,i) = r(n).Sigma(:,:,i) * MuTmp;
	end
	%Create single Gaussian N(MuQ,SigmaQ) based on state sequence q
	[~,r(n).q] = max(model.Pix(:,(n-1)*T1+1:n*T1),[],1); %works also for nbStates=1
	r(n).MuQ = reshape(r(n).Mu(:,r(n).q), model.nbVarPos*model.nbDeriv*T1, 1);
	r(n).SigmaQ = zeros(model.nbVarPos*model.nbDeriv*T1);
	for t=1:T1
		id1 = (t-1)*model.nbVarPos*model.nbDeriv+1:t*model.nbVarPos*model.nbDeriv;
		r(n).SigmaQ(id1,id1) = r(n).Sigma(:,:,r(n).q(t));
	end
	%Retrieval of data with trajectory GMM
	PHIinvSigmaQ = PHI1'/r(n).SigmaQ;
	Rq = PHIinvSigmaQ * PHI1;
	rq = PHIinvSigmaQ * r(n).MuQ;
	r(n).Data = reshape(Rq\rq, model.nbVarPos, T1); %Reshape data for plotting
end


%% Reproduction for new task parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('New reproductions...');
for n=1:nbRepros
	for m=1:model.nbFrames
		%Random generation of new task parameters
		id=ceil(rand(2,1)*nbSamples);
		w=rand(2); w=w/sum(w);
		rnew(n).p(m).b = s(id(1)).p(m).b * w(1) + s(id(2)).p(m).b * w(2);
		rnew(n).p(m).A = s(id(1)).p(m).A * w(1) + s(id(2)).p(m).A * w(2);
	end
	%GMM products
	for i=1:model.nbStates
		SigmaTmp = zeros(model.nbVar);
		MuTmp = zeros(model.nbVar,1);
		for m=1:model.nbFrames
			MuP = rnew(n).p(m).A * model.Mu(:,m,i) + rnew(n).p(m).b;
			SigmaP = rnew(n).p(m).A * model.Sigma(:,:,m,i) * rnew(n).p(m).A';
			SigmaTmp = SigmaTmp + inv(SigmaP);
			MuTmp = MuTmp + SigmaP\MuP;
		end
		rnew(n).Sigma(:,:,i) = inv(SigmaTmp);
		rnew(n).Mu(:,i) = rnew(n).Sigma(:,:,i) * MuTmp;
	end
	%Create single Gaussian N(MuQ,SigmaQ) based on state sequence q
	[~,rnew(n).q] = max(model.Pix(:,1:T1),[],1); %works also for nbStates=1
	rnew(n).MuQ = reshape(rnew(n).Mu(:,rnew(n).q), model.nbVarPos*model.nbDeriv*T1, 1);
	rnew(n).SigmaQ = zeros(model.nbVarPos*model.nbDeriv*T1);
	for t=1:T1
		id1 = (t-1)*model.nbVarPos*model.nbDeriv+1:t*model.nbVarPos*model.nbDeriv;
		rnew(n).SigmaQ(id1,id1) = rnew(n).Sigma(:,:,rnew(n).q(t));
	end
	%Retrieval of data with trajectory GMM
	PHIinvSigmaQ = PHI1'/rnew(n).SigmaQ;
	Rq = PHIinvSigmaQ * PHI1;
	rq = PHIinvSigmaQ * rnew(n).MuQ;
	rnew(n).Data = reshape(Rq\rq, model.nbVarPos, T1); %Reshape data for plotting
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
	plot(s(n).Data0(2,1), s(n).Data0(3,1),'.','markersize',12,'color',clrmap(n,:));
	plot(s(n).Data0(2,:), s(n).Data0(3,:),'-','linewidth',1.5,'color',clrmap(n,:));
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
	plotGMM(r(n).Mu(1:2,:,1), r(n).Sigma(1:2,1:2,:,1), [.5 .5 .5],.8);
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
	plotGMM(rnew(n).Mu(1:2,:,1), rnew(n).Sigma(1:2,1:2,:,1), [.5 .5 .5],.8);
end
axis(limAxes); axis square; set(gca,'xtick',[],'ytick',[]);

%print('-dpng','graphs/demo_trajTPGMM01.png');
%pause;
%close all;


