function demo_GMM01
%Example of Gaussian mixture model (GMM)
%Sylvain Calinon, 2015

addpath('./m_fcts/');

%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model.nbStates = 5; %Number of states in the GMM
model.nbVar = 2; %Number of variables [x1,x2]
nbData = 200; %Length of each trajectory


%% Load AMARSI data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
demos=[];
load('data/AMARSI/GShape.mat');
nbSamples = length(demos);
Data=[];
for n=1:nbSamples
	s(n).Data = spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),nbData)); %Resampling
	Data = [Data s(n).Data]; 
end


%% Parameters estimation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model = init_GMM_kmeans(Data, model);
model = EM_GMM(Data, model);


%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10,10,1000,500]); hold on; box on; 
plotGMM(model.Mu, model.Sigma, [.8 0 0]);
plot(Data(1,:),Data(2,:),'.','markersize',8,'color',[.7 .7 .7]);
axis equal; set(gca,'Xtick',[]); set(gca,'Ytick',[]);

%print('-dpng','graphs/demo_GMM01.png');
%pause;
%close all;
