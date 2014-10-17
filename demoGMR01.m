function demoGMR01
% Gaussian mixture model (GMM) and Gaussian mixture regression (GMR)
%
% Author:	Sylvain Calinon, 2014
%         http://programming-by-demonstration.org/lib/
%
% This source code is given for free! In exchange, please cite the following 
% reference in any academic publication that uses this code or part of it:
%
% @article{Calinon07SMC,
%   author="Calinon, S. and Guenter, F. and Billard, A. G.",
%   title="On Learning, Representing and Generalizing a Task in a Humanoid Robot",
%   journal="{IEEE} Trans. on Systems, Man and Cybernetics, Part {B}",
%   year="2007",
%   volume="37",
%   number="2",
%   pages="286--298",
% }

addpath('./m_fcts/');

%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model.nbVar = 3; %Number of variables 
model.nbStates = 5; %Number of states 
nbData = 250; %Length of each trajectory

%% Load AMARSI data 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
demos=[];
load('data/AMARSI/GShape.mat');
nbSamples = length(demos);
Data=[];
for n=1:nbSamples
  Data = [Data [1:nbData; demos{n}.pos(:,1:4:end)]]; %-> 250 datapoints for each demo
end

%% Learning and reproduction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model = init_GMM_timeBased(Data, model);
model = EM_GMM(Data, model);
DataOut = GMR(model, 1:nbData, 1, [2,3]);

%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[20,100,500,600]); hold on; box on;
plotGMM(model.Mu(2:model.nbVar,:), model.Sigma(2:model.nbVar,2:model.nbVar,:), [0 .8 0]);
plot(Data(2,:),Data(3,:),'.','markersize',8,'color',[.75 .75 .75]);
plot(DataOut(1,:),DataOut(2,:),'-','linewidth',2,'color',[.2 .2 .2]);
axis equal; set(gca,'Xtick',[]); set(gca,'Ytick',[]);

%pause;
%close all;

