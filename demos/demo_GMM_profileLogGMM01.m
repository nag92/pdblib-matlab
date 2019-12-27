function demo_GMM_profileLogGMM01
% Example of univariate velocity profile fitting with a lognormal mixture model and a weighted EM algorithm
% 
% If this code is useful for your research, please cite the related publication:
% @article{Calinon16JIST,
% 	author="Calinon, S.",
% 	title="A Tutorial on Task-Parameterized Movement Learning and Retrieval",
% 	journal="Intelligent Service Robotics",
%		publisher="Springer Berlin Heidelberg",
%		doi="10.1007/s11370-015-0187-9",
%		year="2016",
%		volume="9",
%		number="1",
%		pages="1--29"
% }
%
% Copyright (c) 2019 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon, http://calinon.ch/
% 
% This file is part of PbDlib, http://www.idiap.ch/software/pbdlib/
% 
% PbDlib is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License version 3 as
% published by the Free Software Foundation.
% 
% PbDlib is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with PbDlib. If not, see <http://www.gnu.org/licenses/>.

addpath('./m_fcts/');


%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model.nbStates = 40; %Number of states in the GMM
model.nbVar = 1; %Number of variables [x1]
nbData = 1000; %Length of each trajectory
nbSamples = 1; %Number of samples


%% Load data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% demos=[];
% load('data/2Dletters/W.mat');
% Data=[]; w=[];
% for n=1:nbSamples
% 	s(n).w = spline(1:size(demos{n}.vel(1:model.nbVar,:),2), demos{n}.vel(1:model.nbVar,:), linspace(1,size(demos{n}.pos,2),nbData)); %Resampling
% 	Data = [Data, log(1:nbData)]; %log data are collected
% 	w = [w, s(n).w];
% end
% w = w-min(w);
% w = w/max(w);

% Data = log(1:nbData);
% load('data/lognormal03.mat');

% s = importdata('data/2cold_vel_orig.csv');
% Data = s.data';
% save('data/velprofile01.mat','Data');
% return

load('data/velprofile01.mat');
Data = spline(1:size(Data,2),Data,linspace(1,size(Data,2),nbData)); %Resample data
tlist = Data(1,:) + 1E-8; %The first value should be non-zero
w = Data(2,:);
%Data = tlist;
Data = log(tlist);

%Rescale data and make it positive to represent a probability density function
w = w-min(w);
w = w/max(w);


%% Parameters estimation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%model = init_GMM_timeBased(Data, model);
model = init_GMM_logBased(Data, model);
model = EM_weighted_univariateGMM(Data, w, model);

%Recompute Priors by regression as in RBFN (The resulting priors will represent weights and do not sum to one)
for i=1:model.nbStates
	%Phi(:,i) = gaussPDF(tlist, model.Mu(:,i), model.Sigma(:,:,i)); 
	Phi(:,i) = logGaussPDF(tlist, model.Mu(:,i), model.Sigma(:,:,i)); %not the same as gaussPDF(log(tlist),..)!
end
model.Priors = (Phi'*Phi)\Phi'*w';
%sum(model.Priors)

%Probability density function of lognormal distributions
for i=1:model.nbStates
	%h(i,:) = model.Priors(i) * gaussPDF(tlist, model.Mu(:,i), model.Sigma(:,:,i));
	h(i,:) = model.Priors(i) * logGaussPDF(tlist, model.Mu(:,i), model.Sigma(:,:,i));
end
hmix = sum(h,1);


%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('PaperPosition',[0 0 24 4],'position',[10,10,1200,200]); hold on; box on; 
for i=1:model.nbStates
	plot(tlist, h(i,:),'-','linewidth',4,'color',[1 .7 .7]);
end
for n=1:nbSamples
	plot(tlist,w((n-1)*nbData+1:n*nbData), '-','linewidth',6,'color',[.7 .7 .7]);
end
plot(tlist, hmix,'-','linewidth',2,'color',[.8 0 0]);
axis([tlist(1) tlist(end) 0 1.05]); set(gca,'Xtick',[]); set(gca,'Ytick',[]);
xlabel('$t$','fontsize',18,'interpreter','latex'); 
ylabel('$|\dot{x}|$','fontsize',18,'interpreter','latex');

% w=hmix;
% save('data/lognormal04.mat','w');
% return

disp(['Error: ' num2str(norm(hmix-w))]);

%print('-dpng','graphs/demo_profileLogGMM01.png');
pause;
close all;
