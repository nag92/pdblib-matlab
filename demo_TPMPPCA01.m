function demo_TPMPPCA01
%Example of task-parameterized mixture of probabilistic principal component analyzers (MPPCA) (TP-GMM)
%Sylvain Calinon, 2015

addpath('./m_fcts/');

%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model.nbStates = 3; %Number of Gaussians in the GMM
model.nbFrames = 2; %Number of candidate frames of reference
model.nbVar = 2; %Dimension of the datapoints in the dataset (here: x1,x2)
model.nbFA = 1; %Dimension of the subspace (number of principal components)


%% Load 3rd order tensor data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Load 3rd order tensor data...');
% The MAT file contains a structure 's' with the multiple demonstrations. 's(n).Data' is a matrix data for
% sample n (with 's(n).nbData' datapoints). 's(n).p(m).b' and 's(n).p(m).A' contain the position and
% orientation of the m-th candidate coordinate system for this demonstration. 'Data' contains the observations
% in the different frames. It is a 3rd order tensor of dimension D x P x N, with D=2 the dimension of a
% datapoint, P=2 the number of candidate frames, and N=200x4 the number of datapoints in a trajectory (200)
% multiplied by the number of demonstrations (nbSamples=5).
load('data/Data01.mat');


%% TP-MPPCA learning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('Parameters estimation of TP-MPPCA with EM:');
model = init_tensorGMM_kmeans(Data, model); 
model = EM_tensorMPPCA(Data, model);

%Reconstruct GMM for each demonstration
for n=1:nbSamples
	[s(n).Mu, s(n).Sigma] = productTPGMM0(model, s(n).p);
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
subplot(1,model.nbFrames+1,1); hold on; box on; title('Demonstrations');
for n=1:nbSamples
	%Plot frames
	for m=1:model.nbFrames
		plot([s(n).p(m).b(1) s(n).p(m).b(1)+s(n).p(m).A(1,2)], [s(n).p(m).b(2) s(n).p(m).b(2)+s(n).p(m).A(2,2)], '-','linewidth',6,'color',colPegs(m,:));
		plot(s(n).p(m).b(1), s(n).p(m).b(2),'.','markersize',30,'color',colPegs(m,:)-[.05,.05,.05]);
	end
	%Plot trajectories
	plot(s(n).Data(2,1), s(n).Data(3,1),'.','markersize',15,'color',clrmap(n,:));
	plot(s(n).Data(2,:), s(n).Data(3,:),'-','linewidth',1.5,'color',clrmap(n,:));
	%Plot Gaussians
	plotGMM(s(n).Mu, s(n).Sigma, [.5 .5 .5],.8);
end
axis(limAxes); axis square; set(gca,'xtick',[],'ytick',[]);

%FRAMES
for m=1:model.nbFrames
	subplot(1,model.nbFrames+1,1+m); hold on; grid on; box on; title(['Frame ' num2str(m)]);
	for n=1:nbSamples
		plot(squeeze(Data(1,m,(n-1)*s(1).nbData+1)), ...
			squeeze(Data(2,m,(n-1)*s(1).nbData+1)), '.','markersize',15,'color',clrmap(n,:));
		plot(squeeze(Data(1,m,(n-1)*s(1).nbData+1:n*s(1).nbData)), ...
			squeeze(Data(2,m,(n-1)*s(1).nbData+1:n*s(1).nbData)), '-','linewidth',1.5,'color',clrmap(n,:));
	end
	plotGMM(squeeze(model.Mu(:,m,:)), squeeze(model.Sigma(:,:,m,:)), [.5 .5 .5],.8);
	axis square; set(gca,'xtick',[0],'ytick',[0]);
end

%print('-dpng','graphs/demo_TPMPPCA01.png');
%pause;
%close all;


