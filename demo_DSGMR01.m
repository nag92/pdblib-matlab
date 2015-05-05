function demo_DSGMR01
%Example of Gaussian mixture model (GMM), with Gaussian mixture regression(GMR) and dynamical systems used for reproduction,
%with decay variable used as input (as in DMP)
%Sylvain Calinon, 2015

addpath('./m_fcts/');

%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model.nbStates = 5; %Number of states in the GMM
model.nbVar = 3; %Number of variables [t,x1,x2]
nbData = 200; %Length of each trajectory
model.dt = 0.001; %Time step
nbSamples = 5; %Number of demonstrations

model.kP = 2000; %Stiffness gain
model.kV = (2*model.kP)^.5; %Damping gain (with ideal underdamped damping ratio)
model.alpha = 1.0; %Decay factor
nbVarOut = model.nbVar-1;
L = [eye(nbVarOut)*model.kP, eye(nbVarOut)*model.kV];

%% Load AMARSI data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sIn(1) = 1; %Initialization of decay term
for t=2:nbData
	sIn(t) = sIn(t-1) - model.alpha * sIn(t-1) * model.dt; %Update of decay term (ds/dt=-alpha s)
end

%Store data as [s; x]
demos=[];
load('data/AMARSI/CShape.mat');
Data=[];
for n=1:nbSamples
	s(n).Data = spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),nbData)); %Resampling
	Data = [Data [sIn; s(n).Data]]; 
end


%% Learning and reproduction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%model = init_GMM_timeBased(Data, model);
model = init_GMM_kmeans(Data, model);
model = EM_GMM(Data, model);
[currTar, currSigma] = GMR(model, sIn, 1, 2:3);
%Motion retrieval with spring-damper system
x = Data(2:model.nbVar,1);
dx = zeros(nbVarOut,1);
for t=1:nbData
	%Compute acceleration, velocity and position
	ddx =  -L * [x-currTar(:,t); dx]; %See Eq. (4.0.1) in doc/TechnicalReport.pdf 
	dx = dx + ddx * model.dt;
	x = x + dx * model.dt;
	DataOut(:,t) = x;
end

%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10,10,1000,500]); 
%Plot GMM
subplot(1,2,1); hold on; box on; title('GMM');
plotGMM(model.Mu(2:model.nbVar,:), model.Sigma(2:model.nbVar,2:model.nbVar,:), [.8 0 0]);
plot(Data(2,:),Data(3,:),'.','markersize',8,'color',[.7 .7 .7]);
axis equal; set(gca,'Xtick',[]); set(gca,'Ytick',[]);
%Plot DS-GMR
subplot(1,2,2); hold on; box on; title('DS-GMR');
plotGMM(currTar, currSigma, [0 .8 0]);
plot(Data(2,:),Data(3,:),'.','markersize',8,'color',[.7 .7 .7]);
plot(currTar(1,:),currTar(2,:),'-','linewidth',1.5,'color',[0 .6 0]);
plot(DataOut(1,:),DataOut(2,:),'-','linewidth',3,'color',[0 .3 0]);
axis equal; set(gca,'Xtick',[]); set(gca,'Ytick',[]);

%print('-dpng','graphs/demo_DSGMR01.png');
%pause;
%close all;
