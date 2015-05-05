function demo_trajGMM01
%Example of trajectory synthesis with a GMM with dynamic features (trajectory GMM)
%Sylvain Calinon, 2015

addpath('./m_fcts/');

%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model.nbStates = 5; %Number of components in the GMM
model.nbVarPos = 2; %Dimension of position data (here: x1,x2)
model.nbDeriv = 3; %Number of static&dynamic features (D=2 for [x,dx], D=3 for [x,dx,ddx], etc.)
model.nbVar = model.nbVarPos * model.nbDeriv;
model.dt = 1; %Time step (without rescaling, large values such as 1 has the advantage of creating clusers based on position information)
nbSamples = 4; %Number of demonstrations
nbD = 200; %Number of datapoints in a trajectory

[PHI,PHI1,T,T1] = constructPHI(model,nbD,nbSamples); %Construct PHI operator (big sparse matrix)


%% Load dataset
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('data/dataTrajGMM01.mat');
%load('data/dataTrajGMM02.mat'); %periodic motion
%load('data/dataTrajGMM03.mat'); %motion with options
%load('data/dataTrajGMM04.mat'); %partial demonstrations

%Resampling of dataset
nbD0 = size(Data,2)/nbSamples; %Original number of datapoints in a trajectory
DataTmp = [];
for n=1:nbSamples
	DataTmp = [DataTmp spline(1:nbD0, Data(2:model.nbVarPos+1,(n-1)*nbD0+1:n*nbD0), linspace(1,nbD0,nbD))]; 
end
Data = DataTmp;

%Re-arrange data in vector form
x = reshape(Data(:,1:T), model.nbVarPos*T, 1) * 1E2; %Scale data to avoid numerical computation problem
zeta = PHI*x; %y is for example [x1(1), x2(1), x1d(1), x2d(1), x1(2), x2(2), x1d(2), x2d(2), ...], see Eq. (2.4.5) in doc/TechnicalReport.pdf
Data = reshape(zeta, model.nbVarPos*model.nbDeriv, T); %Include derivatives in Data


%% Parameters estimation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Initialization with kmeans
model = init_GMM_kmeans(Data, model);

% %Initialization with homogeneous time intervals
% DataTS = [repmat([1:T1]*model.dt,1,nbSamples); Data];
% modelTmp = init_GMM_timeBased(DataTS, model);
% model.Mu = modelTmp.Mu(2:end,:);
% model.Sigma = modelTmp.Sigma(2:end,2:end,:);
% model.Priors = modelTmp.Priors;

%Refinement of parameters
[model, GAMMA2] = EM_GMM(Data, model);
%[model, GAMMA2] = EM_MFA(Data, model);
%[model, GAMMA2] = EM_MPPCA(Data, model);
%[model, GAMMA2] = EM_HDGMM(Data, model);

%Precomputation of inverses (optional)
for i=1:model.nbStates
	model.invSigma(:,:,i) = inv(model.Sigma(:,:,i));
end


%% Reproduction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for n=1:1 %nbSamples
	%Compute best path for the n-th demonstration
	[~,r(n).q] = max(GAMMA2(:,(n-1)*T1+1:n*T1),[],1); %works also for nbStates=1
	
	%Create single Gaussian N(MuQ,SigmaQ) based on optimal state sequence q, see Eq. (2.4.8) in doc/TechnicalReport.pdf
	MuQ = reshape(model.Mu(:,r(n).q), model.nbVar*T1, 1); 
	%MuQ = zeros(model.nbVar*T1,1);
	SigmaQ = zeros(model.nbVar*T1);
	for t=1:T1
		id = (t-1)*model.nbVar+1:t*model.nbVar;
		%MuQ(id) = model.Mu(:,r(n).q(t)); 
		SigmaQ(id,id) = model.Sigma(:,:,r(n).q(t));
	end
	%SigmaQ can alternatively be computed with:
	%r(n).SigmaQ = (kron(ones(T1,1), eye(model.nbVar)) * reshape(model.Sigma(:,:,r(1).q), model.nbVar, model.nbVar*T1)) .* kron(eye(T1), ones(model.nbVar));


	%Least squares computation method 1 (using lscov Matlab function), see Eq. (2.4.11) in doc/TechnicalReport.pdf
	%%%%%%%%%%%%%%%%%%%
	[zeta,~,mse,S] = lscov(PHI1, MuQ, SigmaQ,'chol'); %Retrieval of data with weighted least squares solution
	r(n).Data = reshape(zeta, model.nbVarPos, T1); %Reshape data for plotting


% 	%Least squares computation method 2 (most readable but not optimized), see Eq. (2.4.11) in doc/TechnicalReport.pdf
% 	%%%%%%%%%%%%%%%%%%%
% 	PHIinvSigmaQ = PHI1' / SigmaQ;
% 	Rq = PHIinvSigmaQ * PHI1;
% 	rq = PHIinvSigmaQ * MuQ;
% 	zeta = Rq \ rq; %Can also be computed with c = lscov(Rq, rq)
% 	r(n).Data = reshape(zeta, model.nbVarPos, T1); %Reshape data for plotting
% 	%Covariance Matrix computation of ordinary least squares estimate, see Eq. (2.4.12) in doc/TechnicalReport.pdf
% 	mse =  (MuQ'*inv(SigmaQ)*MuQ - rq'*inv(Rq)*rq)  ./ ((model.nbVar-model.nbVarPos)*T1);
% 	S = inv(Rq) * mse; 


% 	%Least squares computation method 3 (efficient computation using Cholesky and QR decompositions, inspired by lscov code)
% 	%%%%%%%%%%%%%%%%%%%
% 	T = chol(SigmaQ)'; %SigmaQ=T*T'
% 	TA = T \ PHI1;
% 	TMuQ = T \ MuQ;
% 	[Q, R, perm] = qr(TA,0); %PHI1(:,perm)=Q*R
% 	z = Q' * TMuQ;
% 	zeta(perm,:) = R \ z; %zeta=(TA'*TA)\(TA'*TMuQ), see Eq. (2.4.11) in doc/TechnicalReport.pdf
% 	r(n).Data = reshape(zeta, model.nbVarPos, T1); %Reshape data for plotting
% 	%Covariance Matrix computation of ordinary least squares estimate
% 	err = TMuQ - Q*z;
% 	mse = err'*err ./ (model.nbVar*T1 - model.nbVarPos*T1);
% 	Rinv = R \ eye(model.nbVarPos*T1);
% 	S(perm,perm) = Rinv*Rinv' .* mse; %See Eq. (2.4.12) in doc/TechnicalReport.pdf
	
	
	%Rebuild covariance by reshaping S, see Eq. (2.4.12) in doc/TechnicalReport.pdf
	for t=1:T1
		id = (t-1)*model.nbVarPos+1:t*model.nbVarPos;
		r(n).expSigma(:,:,t) = S(id,id) * T1;
	end
	
end %nbSamples


%% Plot timeline
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10 10 600 600]);
for m=1:model.nbVarPos
	limAxes = [1, T1, min(Data(m,:))-1E0, max(Data(m,:))+1E0];
	subplot(model.nbVarPos,1,m); hold on;
	for n=1:1 %nbSamples
		msh=[]; x0=[];
		for t=1:T1-1
			if size(msh,2)==0
				msh(:,1) = [t; model.Mu(m,r(n).q(t))];
			end
			if t==T1-1 || r(n).q(t+1)~=r(n).q(t)
				msh(:,2) = [t+1; model.Mu(m,r(n).q(t))];
				sTmp = model.Sigma(m,m,r(n).q(t))^.5;
				msh2 = [msh(:,1)+[0;sTmp], msh(:,2)+[0;sTmp], msh(:,2)-[0;sTmp], msh(:,1)-[0;sTmp], msh(:,1)+[0;sTmp]];
				patch(msh2(1,:), msh2(2,:), [.85 .85 .85],'edgecolor',[.7 .7 .7]);
				plot(msh(1,:), msh(2,:), '-','linewidth',3,'color',[.7 .7 .7]);
				plot([msh(1,1) msh(1,1)], limAxes(3:4), ':','linewidth',1,'color',[.7 .7 .7]);
				x0 = [x0 msh];
				msh=[];
			end
		end
		msh = [1:T1, T1:-1:1; r(n).Data(m,:)-squeeze(r(n).expSigma(m,m,:).^.5)'*1, fliplr(r(n).Data(m,:)+squeeze(r(n).expSigma(m,m,:).^.5)'*1)];
		patch(msh(1,:), msh(2,:), [1 .4 .4],'edgecolor',[1 .7 .7],'edgealpha',.8,'facealpha',.8);
	end
	for n=1:nbSamples
		plot(1:T1, Data(m,(n-1)*T1+1:n*T1), '-','lineWidth',2,'color',[.2 .2 .2]);
	end
	for n=1:1
		plot(1:T1, r(n).Data(m,:), '-','lineWidth',2.5,'color',[.8 0 0]);
	end
	
	set(gca,'xtick',[],'ytick',[]);
	xlabel('$t$','interpreter','latex','fontsize',18);
	ylabel(['$x_' num2str(m) '$'],'interpreter','latex','fontsize',18);
	axis(limAxes);
end

%% Plot 2D
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if model.nbVarPos>1
	figure('position',[620 10 600 600]); hold on;
	for n=1:1 %nbSamples
		plotGMM(r(n).Data([1,2],:), r(n).expSigma([1,2],[1,2],:)*2, [1 .2 .2]);
	end
	plotGMM(model.Mu([1,2],:), model.Sigma([1,2],[1,2],:)*2, [.5 .5 .5],.8);
	for n=1:nbSamples
		plot(Data(1,(n-1)*T1+1:n*T1), Data(2,(n-1)*T1+1:n*T1), '-','lineWidth',2,'color',[.2 .2 .2]); %-0.2+0.8*(n-1)/(nbSamples-1)
	end
	for n=1:1
		plot(r(n).Data(1,:), r(n).Data(2,:), '-','lineWidth',2.5,'color',[.8 0 0]);
	end
	set(gca,'xtick',[],'ytick',[]); axis equal; axis square;
	xlabel(['$x_1$'],'interpreter','latex','fontsize',18);
	ylabel(['$x_2$'],'interpreter','latex','fontsize',18);
end

%print('-dpng','graphs/demo_trajGMM01.png');
%pause;
%close all;


