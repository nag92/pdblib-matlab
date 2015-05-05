function benchmark_DS_GP_raw02
%Benchmark of task-parameterized model based on Gaussian process regression, 
%with raw trajectory, and spring-damper system used for reproduction
%Sylvain Calinon, 2015

addpath('./m_fcts/');

%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model.nbFrames = 2; %Number of candidate frames of reference
model.nbVar = 2; %Dimension of the datapoints in the dataset (here: x1,x2)
model.dt = 0.01; %Time step
model.kP = 100; %Stiffness gain
model.kV = (2*model.kP)^.5; %Damping gain (with ideal underdamped damping ratio)
nbRepros = 20; %Number of reproductions with new situations randomly generated
L = [eye(model.nbVar)*model.kP, eye(model.nbVar)*model.kV];

%% Load 3rd order tensor data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Load 3rd order tensor data...');
% The MAT file contains a structure 's' with the multiple demonstrations. 's(n).Data' is a matrix data for
% sample n (with 's(n).nbData' datapoints). 's(n).p(m).b' and 's(n).p(m).A' contain the position and
% orientation of the m-th candidate coordinate system for this demonstration. 'Data' contains the observations
% in the different frames. It is a 3rd order tensor of dimension D x P x N, with D=3 the dimension of a
% datapoint, P=2 the number of candidate frames, and N=200x4 the number of datapoints in a trajectory (200)
% multiplied by the number of demonstrations (5).
load('data/DataLQR01.mat');


%% Transformation of 'Data' to learn the path of the spring-damper system
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nbD = s(1).nbData;
%Create transformation matrix to compute [X; DX; DDX]
D = (diag(ones(1,nbD-1),-1)-eye(nbD)) / model.dt;
D(end,end) = 0;
%Create transformation matrix to compute XHAT = X + DX*kV/kP + DDX/kP
K1d = [1, model.kV/model.kP, 1/model.kP];
K = kron(K1d,eye(model.nbVar));
%Create 3rd order tensor data with XHAT instead of X, see Eq. (4.0.2) in doc/TechnicalReport.pdf
%Data = zeros(model.nbVar, model.nbFrames, nbD*nbSamples);
Data = s(1).Data0(1,:);
for n=1:nbSamples
	DataTmp = s(n).Data0(2:end,:);
	s(n).Data = K * [DataTmp; DataTmp*D; DataTmp*D*D];
	Data = [Data; s(n).Data]; %Data is a matrix of size M*D x T (stacking the different trajectory samples)
end

%% GPR with raw trajectory encoding
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('Parameters estimation of GPR with raw trajectory encoding:');
for n=1:nbSamples
	%Set query point vector (position and orientation of the two objects)
	s(n).DataIn = [s(n).p(1).b(2:3); s(n).p(1).A(2:3,3); s(n).p(2).b(2:3); s(n).p(2).A(2:3,3)];
	model.DataIn(:,n) = s(n).DataIn;
	%Set model output vector (raw trajectory data)
	model.DataOut(:,n) = reshape(s(n).Data, model.nbVar*nbD, 1);
end


% %% Reproduction with GPR for the task parameters used to train the model
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% disp('Reproductions with spring-damper system...');
% for n=1:nbSamples
% 	%Direct retrieval of attractor path through GPR
% 	vOut = GPR(model.DataIn, model.DataOut, s(n).DataIn);
% 	currTar = reshape(vOut, model.nbVar, nbD);
% 	
% 	%Motion retrieval with spring-damper system
% 	x = s(n).p(1).b(2:3);
% 	dx = zeros(model.nbVar,1);
% 	for t=1:s(n).nbData
% 		%Compute acceleration, velocity and position
% 		ddx =  -L * [x-currTar(:,t); dx]; %See Eq. (4.0.1) in doc/TechnicalReport.pdf
% 		dx = dx + ddx * model.dt;
% 		x = x + dx * model.dt;
% 		r(n).Data(:,t) = x;
% 	end
% end


%% Reproduction with GPR for new task parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('New reproductions with spring-damper system...');

% %Random generation of new task parameters
% for n=1:10
% 	id=ceil(rand(2,1)*nbSamples);
% 	w=rand(2); w=w/sum(w);
% 	taskParams(n).p(1) = s(1).p(1);
% 	taskParams(n).p(2).b = s(id(1)).p(2).b * w(1) + s(id(2)).p(2).b * w(2);
% 	taskParams(n).p(2).A = s(id(1)).p(2).A * w(1) + s(id(2)).p(2).A * w(2);
% end
% for n=11:20
%   taskParams(n).p(1) = s(1).p(1);
% 	taskParams(n).p(2).b = [0; rand(1,1) * 2; rand(1,1)];
% 	aTmp = rand(1) * pi + pi;
% 	taskParams(n).p(2).A = [[1;0;0] [0;cos(aTmp);-sin(aTmp)] [0;sin(aTmp);cos(aTmp)]];
% 	taskParams(n).p(2).A(2:end,2:end) = taskParams(n).p(2).A(2:end,2:end) * norm(s(1).p(1).A(:,2));
% end
% save('data/taskParams2.mat','taskParams');

load('data/taskParams2.mat'); %Load new task parameters (new situation)

	
for n=1:nbRepros
	rnew(n).p = taskParams(n).p;
	%Query point vector (position and orientation of the two objects)
	rnew(n).DataIn = [rnew(n).p(1).b(2:3); rnew(n).p(1).A(2:3,3); rnew(n).p(2).b(2:3); rnew(n).p(2).A(2:3,3)];
	
	%Direct retrieval of attractor path through GPR
	[vOut, vOutSigma] = GPR(model.DataIn, model.DataOut, rnew(n).DataIn, [5E-1, 1E-1, 1E-2]);
	rnew(n).currTar  = reshape(vOut, model.nbVar, nbD);
	for t=1:nbD
		id = (t-1)*model.nbVar+1:t*model.nbVar;
		%id = t:t+nbD:nbD*model.nbVar;
		rnew(n).currSigma(:,:,t) = vOutSigma(id,id) / 20;
	end

	%Motion retrieval with spring-damper system
	x = rnew(n).p(1).b(2:3);
	dx = zeros(model.nbVar,1);
	for t=1:nbD
		%Compute acceleration, velocity and position
		ddx =  -L * [x-rnew(n).currTar(:,t); dx]; %See Eq. (4.0.1) in doc/TechnicalReport.pdf 
		dx = dx + ddx * model.dt;
		x = x + dx * model.dt;
		rnew(n).Data(:,t) = x;
	end
end


%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('PaperPosition',[0 0 4 3],'position',[20,50,600,450]);
axes('Position',[0 0 1 1]); axis off; hold on;
set(0,'DefaultAxesLooseInset',[0,0,0,0]);
limAxes = [-1.5 2.5 -1.6 1.4]*.8;
myclr = [0.2863 0.0392 0.2392; 0.9137 0.4980 0.0078; 0.7412 0.0824 0.3137];

%Plot demonstrations
plotPegs(s(1).p(1), myclr(1,:), .1);
for n=1:nbSamples
	plotPegs(s(n).p(2), myclr(2,:), .1);
	patch([s(n).Data0(2,1:end) s(n).Data0(2,end:-1:1)], [s(n).Data0(3,1:end) s(n).Data0(3,end:-1:1)],...
		[1 1 1],'linewidth',1.5,'edgecolor',[0 0 0],'facealpha',0,'edgealpha',0.04);
end
axis equal; axis(limAxes);

%Plot reproductions in new situations
h=[];
for n=1:nbRepros
	delete(h);
	h = plotPegs(rnew(n).p);
	%h = [h plotGMM(rnew(n).currTar, rnew(n).currSigma,  [0 .8 0], .2)];
	h = [h patch([rnew(n).Data(1,:) rnew(n).Data(1,fliplr(1:nbD))], [rnew(n).Data(2,:) rnew(n).Data(2,fliplr(1:nbD))],...
		[1 1 1],'linewidth',1.5,'edgecolor',[0 0 0],'facealpha',0,'edgealpha',0.4)];
	h = [h plot(rnew(n).Data(1,1), rnew(n).Data(2,1),'.','markersize',12,'color',[0 0 0])];
	axis equal; axis(limAxes);
	print('-dpng','-r600',['graphs/benchmark_DS_GP_raw_intro' num2str(n,'%.2d') '.png']);
	%pause
end

pause;
close all;

end

%Function to plot pegs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function h = plotPegs(p, colPegs, fa)
if ~exist('colPegs')
	colPegs = [0.2863 0.0392 0.2392; 0.9137 0.4980 0.0078];
	fa = 0.4;
end
pegMesh = [-4 -3.5; -4 10; -1.5 10; -1.5 -1; 1.5 -1; 1.5 10; 4 10; 4 -3.5; -4 -3.5]' *1E-1;
for m=1:length(p)
	dispMesh = p(m).A(2:3,2:3) * pegMesh + repmat(p(m).b(2:3),1,size(pegMesh,2));
	h(m) = patch(dispMesh(1,:),dispMesh(2,:),colPegs(m,:),'linewidth',1,'edgecolor','none','facealpha',fa);
end
end



