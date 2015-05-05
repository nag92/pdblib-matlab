function demo_MFA01
%Example of mixture of factor analysers (MFA) 
%Sylvain Calinon, 2015

addpath('./m_fcts/');

%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model.nbStates = 4; %Number of states in the GMM
model.nbVar = 4; %Number of variables [x1,x2,x3,x4]
model.nbFA = 1; %Dimension of the subspace (number of factor analyzers)
nbData = 200; %Length of each trajectory
nbSamples = 5; %Number of demonstrations


%% Load AMARSI data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
demos=[];
load('data/AMARSI/GShape.mat'); %Load x1,x2 variables
for n=1:nbSamples
	s(n).Data = spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),nbData)); %Resampling
end
demos=[];
load('data/AMARSI/CShape.mat'); %Load x3,x4 variables
Data=[];
for n=1:nbSamples
	s(n).Data = [s(n).Data; spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),nbData))]; %Resampling
	Data = [Data s(n).Data]; 
end


%% Parameters estimation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model = init_GMM_kmeans(Data, model);
model0 = EM_GMM(Data, model); %for comparison
model = EM_MFA(Data, model);


%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10,10,1000,500]); 
for i=1:2
	subplot(1,2,i); hold on; box on; 
	plotGMM(model0.Mu((i-1)*2+1:i*2,:), model0.Sigma((i-1)*2+1:i*2,(i-1)*2+1:i*2,:), [.8 .8 .8]);
	plotGMM(model.Mu((i-1)*2+1:i*2,:), model.Sigma((i-1)*2+1:i*2,(i-1)*2+1:i*2,:), [.8 0 0]);
	plot(Data((i-1)*2+1,:),Data(i*2,:),'.','markersize',8,'color',[.7 .7 .7]);
	axis equal; set(gca,'Xtick',[]); set(gca,'Ytick',[]);
	xlabel(['x_' num2str((i-1)*2+1)]); ylabel(['x_' num2str(i*2)]);
end

%print('-dpng','graphs/demo_MFA01.png');
%pause;
%close all;
