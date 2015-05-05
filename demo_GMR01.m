function demo_GMR01
%Example of Gaussian mixture model (GMM) and time-based Gaussian mixture regression (GMR) used for reproduction
%Sylvain Calinon, 2015

addpath('./m_fcts/');

%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model.nbStates = 5; %Number of states in the GMM
model.nbVar = 3; %Number of variables [t,x1,x2]
model.dt = 0.001; %Time step duration
nbData = 200; %Length of each trajectory
nbSamples = 5; %Number of demonstrations

%% Load AMARSI data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
demos=[];
load('data/AMARSI/CShape.mat');
Data=[];
for n=1:nbSamples
	s(n).Data = spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),nbData)); %Resampling
	Data = [Data [[1:nbData]*model.dt; s(n).Data]]; 
end


%% Learning and reproduction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%model = init_GMM_kmeans(Data, model);
model = init_GMM_timeBased(Data, model);
model = EM_GMM(Data, model);
[DataOut, SigmaOut] = GMR(model, [1:nbData]*model.dt, 1, 2:3);


%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10,10,1000,500]); 
%Plot GMM
subplot(1,2,1); hold on; box on; title('GMM');
plotGMM(model.Mu(2:model.nbVar,:), model.Sigma(2:model.nbVar,2:model.nbVar,:), [.8 0 0]);
plot(Data(2,:),Data(3,:),'.','markersize',8,'color',[.7 .7 .7]);
axis equal; set(gca,'Xtick',[]); set(gca,'Ytick',[]);
%Plot GMR
subplot(1,2,2); hold on; box on; title('GMR');
plotGMM(DataOut, SigmaOut, [0 .8 0]);
plot(Data(2,:),Data(3,:),'.','markersize',8,'color',[.7 .7 .7]);
plot(DataOut(1,:),DataOut(2,:),'-','linewidth',2,'color',[0 .4 0]);
axis equal; set(gca,'Xtick',[]); set(gca,'Ytick',[]);

%print('-dpng','graphs/demo_GMR01.png');
%pause;
%close all;
