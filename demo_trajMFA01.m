function demo_trajMFA01
% Trajectory model with either (see lines 69-71): 
%  -a mixture of factor analysers (MFA)
%  -a mixture of probabilistic principal component analyzers (MPPCA)
%  -a high-fimensional data clustering approach proposed by Bouveyron (HD-GMM)
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model.nbStates = 3; %Number of components in the mixture model
model.nbFA = 1; %Dimension of the subspace in each cluster
model.nbVarPos = 4; %Dimension of the position datapoint
model.nbDeriv = 3; %Nb derivatives+1 (D=2 for [x,dx], D=3 for [x,dx,ddx], etc.)
model.nbVar = model.nbVarPos * model.nbDeriv;
model.dt = 1; %Time step (large values such as 1 will tend to create clusers by following position information)
nbD = 100; %Number of datapoints in a trajectory
nbSamples = 1; %Number of trajectory samples

%Construct operator A (big sparse matrix)
T1 = nbD; %Number of datapoints in a demonstration
T = T1 * nbSamples; %Total number of datapoints
op1D = zeros(model.nbDeriv);
op1D(1,end) = 1;
for i=2:model.nbDeriv
	op1D(i,:) = (op1D(i-1,:) - circshift(op1D(i-1,:),[0,-1])) / model.dt;
end
op = zeros(T1*model.nbDeriv,T1);
op((model.nbDeriv-1)*model.nbDeriv+1:model.nbDeriv*model.nbDeriv,1:model.nbDeriv) = op1D;
A0 = zeros(T1*model.nbDeriv,T1);
for t=0:T1-model.nbDeriv
	A0 = A0 + circshift(op, [model.nbDeriv*t,t]);
end
%Handling of borders
for i=1:model.nbDeriv-1
	op(model.nbDeriv*model.nbDeriv+1-i,:)=0; op(:,i)=0;
	A0 = A0 + circshift(op, [-i*model.nbDeriv,-i]);
end
%Application to multiple dimensions and multiple demonstrations
A1 = kron(A0, eye(model.nbVarPos));
A = kron(eye(nbSamples), A1);


%% Load data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Data = grabDataFromCursor;
% save('data/dataTrajGMM05.mat','Data');

load('data/data03.mat');

%Resampling
nbD0 = size(Data,2)/nbSamples; %Original number of datapoints in a trajectory
DataTmp = [];
for n=1:nbSamples
	DataTmp = [DataTmp spline(1:nbD0, Data(:,(n-1)*nbD0+1:n*nbD0), linspace(1,nbD0,nbD))]; 
end
Data = DataTmp;

%Compute derivatives [x;dx;ddx]
D = (diag(ones(1,nbD-1),-1)-eye(nbD)) / model.dt;
D(end,end) = 0;
for k=1:model.nbDeriv-1
	Data = [Data; Data(1:model.nbVarPos,:)*D^k]; %Compute derivatives
end

%Re-arrange data in vector form
x = reshape(Data(1:model.nbVarPos,:), model.nbVarPos*T, 1) * 1E2; %1E2 to avoid numerical problem
y = A*x; %y is for example [x1(1), x2(1), x1d(1), x2d(1), x1(2), x2(2), x1d(2), x2d(2), ...]
Data = reshape(y, model.nbVar, T); %Include derivatives in Data


%% Learning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model = init_GMM_kmeans(Data, model);

[model, GAMMA2] = EM_GMM(Data, model);
%[model, GAMMA2] = EM_MFA(Data, model);
%[model, GAMMA2] = EM_MPPCA(Data, model);
%[model, GAMMA2] = EM_HDGMM(Data, model);


%% Batch reconstruction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[~,r(1).q] = max(GAMMA2,[],1); %works also for nbStates=1
%Create single Gaussian N(MuQ,SigmaQ) based on state sequence q
r(1).MuQ = reshape(model.Mu(:,r(1).q), model.nbVar*T1, 1);
r(1).SigmaQ = zeros(model.nbVar*T1);
for t=1:T1
	id1 = (t-1)*model.nbVar+1:t*model.nbVar;
	r(1).SigmaQ(id1,id1) = model.Sigma(:,:,r(1).q(t));
end
%r(1).SigmaQ = (kron(ones(T1,1), eye(model.nbVar)) * reshape(model.Sigma(:,:,r(1).q), model.nbVar, model.nbVar*T1)) ...
%		.* kron(eye(T1), ones(model.nbVar));

% %Retrieval of data with trajectory GMM (non-optimized computation)
% AinvSigmaQ = A1'/r(1).SigmaQ;
% Rq = AinvSigmaQ * A1;
% rq = AinvSigmaQ * r(1).MuQ;
% r(1).Data = reshape(Rq\rq, model.nbVarPos, T1); %Reshape data for plotting

%Retrieval of data with weighted least squares solution
[c,~,~,S] = lscov(A1, r(1).MuQ, r(1).SigmaQ, 'chol');
r(n).Data = reshape(c, model.nbVarPos, T1); %Reshape data for plotting	
for t=1:T1
	id = (t-1)*model.nbVarPos+1:t*model.nbVarPos;
	r(1).expSigma(:,:,t) = S(id,id) * T1;
end


% %% Iterative batch reconstruction (with recursive update of matrix inversion)
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% disp('Iterative reconstruction');
% [~,r(1).q] = max(GAMMA2,[],1); %works also for nbStates=1
% r(1).MuQ = reshape(model.Mu(:,r(1).q), model.nbVar*T1, 1);
% r(1).invSigmaQ = zeros(model.nbVar*T1);
% for t=1:T1
% 	idc = (t-1)*model.nbVar+1:t*model.nbVar;
% 	r(1).invSigmaQ(idc,idc) = inv(model.Sigma(:,:,r(1).q(t)));
% end
% for t=1:T1
% 	idc2 = (t-1)*model.nbVarPos+1:t*model.nbVarPos;
% 	idc20 = 1:t*model.nbVarPos;
% 	fprintf('.');
% 	if t==1
% 		invR = inv(A1(:,idc2)' * r(1).invSigmaQ * A1(:,idc2));
% 	else
% 		xrTmp = A1(:,idc20)' * r(1).invSigmaQ * A1(:,idc2);
% 		invR = invUpdateSym(invR, xrTmp(1:end-model.nbVarPos,:), xrTmp(end-model.nbVarPos+1:end,:));
% 	end
% end
% fprintf('\n');
% rTmp = A1' * r(1).invSigmaQ * r(1).MuQ;
% r(1).Data = reshape(invR*rTmp, model.nbVarPos, T1); %Reshape data for plotting


% %% Iterative reconstruction (Tokuda, 1995) <- Not working yet
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% disp('Iterative reconstruction');
% [~,r(1).q] = max(GAMMA2,[],1); %works also for nbStates=1
% r(1).MuQ = reshape(model.Mu(:,r(1).q), model.nbVar*T1, 1);
% r(1).invSigmaQ = zeros(model.nbVar*T1);
% for t=1:T1
% 	idp = max(t-2,0)*model.nbVar+1:max(t-1,1)*model.nbVar;
% 	idc = (t-1)*model.nbVar+1:t*model.nbVar;
% 	r(1).invSigmaQ(idc,idc) = inv(model.Sigma(:,:,r(1).q(t)));
% 	fprintf('.');
% 	if t==1
% % 		Ptmp = inv(A1(idc,:)' * r(1).invSigmaQ(idc,idc) * A1(idc,:));
% % 		cTmp = Ptmp * (A1(idc,:)' * r(1).invSigmaQ(idc,idc) * r(1).MuQ(idc));
% % 		Ptmp = zeros(model.nbVarPos*T1);
% % 		cTmp = zeros(model.nbVarPos*T1,1);
% 		Ptmp = inv(r(1).invSigmaQ(:,:));
% 		cTmp = r(1).MuQ;
% 	else
% 		piTmp = Ptmp * A1(idc,:)';
% 		nuTmp = A1(idc,:) * piTmp;
% 		kTmp = piTmp / (eye(model.nbVar) + (r(1).invSigmaQ(idc,idc) - r(1).invSigmaQ(idp,idp)) * nuTmp);
% 		cTmp = cTmp + kTmp * (r(1).invSigmaQ(idc,idc) * (r(1).MuQ(idc) - A1(idc,:)*cTmp) - r(1).invSigmaQ(idp,idp) * (r(1).MuQ(idp) - A1(idc,:)*cTmp));
% 		Ptmp = Ptmp - kTmp * (r(1).invSigmaQ(idc,idc) - r(1).invSigmaQ(idp,idp)) * piTmp';
% 	end
% end
% r(1).Data = reshape(cTmp, model.nbVarPos, T1); %Reshape data for plotting


% %% Iterative reconstruction (Tokuda, 1995) <- Not working yet
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% disp('Iterative reconstruction');
% [~,r(1).q] = max(GAMMA2,[],1); %works also for nbStates=1
% r(1).MuQ = reshape(model.Mu(:,r(1).q), model.nbVar*T1, 1);
% r(1).invSigmaQ = zeros(model.nbVar*T1);
% for t=1:T1
% 	idp = max(t-2,0)*model.nbVar+1:max(t-1,1)*model.nbVar;
% 	idc = (t-1)*model.nbVar+1:t*model.nbVar;
% 	idc2 = (t-1)*model.nbVarPos+1:t*model.nbVarPos;
% 	r(1).invSigmaQ(idc,idc) = inv(model.Sigma(:,:,r(1).q(t)));
% 	fprintf('.');
% 	if t==1
% 		Rtmp = A1(idc,idc2)' * r(1).invSigmaQ(idc,idc) * A1(idc,idc2);
%  		rTmp = A1(idc,idc2)' * r(1).invSigmaQ(idc,idc) * r(1).MuQ(idc);
% 		%Rtmp = zeros(model.nbVarPos); %A1(idc,:)' * r(1).invSigmaQ(idc,idc) * A1(idc,:);
%  		%rTmp = zeros(model.nbVarPos,1); %A1(idc,:)' * r(1).invSigmaQ(idc,idc) * r(1).MuQ(idc);
% 		%Rtmp = inv(r(1).invSigmaQ(idc,idc));
%  		%rTmp = r(1).MuQ(idc);
% 	else
% 		Dtmp = r(1).invSigmaQ(idc,idc) - r(1).invSigmaQ(idp,idp);
% 		dTmp = r(1).invSigmaQ(idc,idc) * r(1).MuQ(idc) - r(1).invSigmaQ(idp,idp) * r(1).MuQ(idp);		
% 		Rtmp = Rtmp + A1(idc,idc2)' * Dtmp * A1(idc,idc2);
% 		rTmp = rTmp + A1(idc,idc2)' * dTmp; 
% 	end
% 	r(1).Data(:,t) = reshape(Rtmp\rTmp, model.nbVarPos, 1);
% end


%% Plot 2D
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[20,10,1200,600]);

subplot(1,2,1); hold on;
plotGMM(r(1).Data([1,2],:), r(1).expSigma([1,2],[1,2],:), [.5 .8 1]);
plotGMM(model.Mu(1:2,:),model.Sigma(1:2,1:2,:),[.8 0 0]);
% plotGMM(model.Mu(1:2,:),model.P(1:2,1:2,:),[1 .4 .4]);
% for i=1:model.nbStates
% 	plotGMM(model.Mu(1:2,i), model.L(1:2,:,i)*model.L(1:2,:,i)'+eye(2)*1E-1, [1 .4 .4]);
% end
plot(Data(1,:), Data(2,:),'.','linewidth',2,'color',[0,0,0]);
plot(r(1).Data(1,:), r(1).Data(2,:),'-','linewidth',2,'color',[.8,0,0]);
axis equal;
xlabel('x_1','fontsize',16); ylabel('x_2','fontsize',16);

subplot(1,2,2); hold on;
plotGMM(r(1).Data([3,4],:), r(1).expSigma([3,4],[3,4],:), [.5 .8 1]);
plotGMM(model.Mu(3:4,:),model.Sigma(3:4,3:4,:),[.8 0 0]);
% plotGMM(model.Mu(3:4,:),model.P(3:4,3:4,:),[1 .4 .4]);
% for i=1:model.nbStates
% 	plotGMM(model.Mu(3:4,i), model.L(3:4,:,i)*model.L(3:4,:,i)'+eye(2)*1E-1, [1 .4 .4]);
% end
plot(Data(3,:), Data(4,:),'.','linewidth',2,'color',[0,0,0]);
plot(r(1).Data(3,:), r(1).Data(4,:),'-','linewidth',2,'color',[.8,0,0]);
axis equal;
xlabel('x_3','fontsize',16); ylabel('x_4','fontsize',16);

%print('-dpng','graphs/demo_trajMFA01.png');
pause;
close all;

end

%% Recursive update of matrix inversion fct
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function M = invUpdateSym(A,x,r)
	xA = x' * A;
	q = eye(size(r)) / (r - xA * x);
	Ax = A * x * q;
	M = [A+Ax*xA, -Ax; -q*xA, q];
end

