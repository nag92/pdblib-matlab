function demo_GMM_profileGMM_multivariate01
% Example of multivariate velocity profile fitting with a Gaussian mixture model (GMM) and a weighted EM algorithm
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
model.nbStates = 4; %Number of states in the GMM
model.nbVar = 2; %Number of variables [x1]
nbData = 200; %Length of each trajectory
nbSamples = 1; %Number of samples


%% Load data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% demos=[];
% load('data/2Dletters/W.mat');
% Data=[]; w=[];
% for n=1:nbSamples
% 	s(n).w = spline(1:size(demos{n}.vel(1:model.nbVar,:),2), demos{n}.vel(1:model.nbVar,:), linspace(1,size(demos{n}.pos,2),nbData)); %Resampling
% 	Data = [Data, repmat(1:nbData,model.nbVar,1)];
% 	w = [w, s(n).w];
% end
% w = w - repmat(min(w,[],2),1,nbData*nbSamples);
% w = w ./ repmat(max(w,[],2),1,nbData*nbSamples);

Data = repmat(1:nbData,model.nbVar,1);
load('data/lognormal05.mat'); %load 'w'


%% Parameters estimation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model = init_GMM_timeBased(Data, model);
model.Priors = repmat(model.Priors,model.nbVar,1);
model = EM_weighted_multivariateGMM(Data, w, model);


%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10,10,1000,500]); 
for k=1:model.nbVar
	subplot(model.nbVar,1,k); hold on; box on; 
	mtmp.nbStates = model.nbStates; 
	mtmp.Priors = model.Priors(k,:);
	mtmp.Mu(1,:) = model.Mu(k,:);
	mtmp.Sigma(1,1,:) = model.Sigma(k,k,:);
	[~,hmix(k,:)] = plotGMM1D(mtmp, [1 nbData 0 1], [.8 0 0], .2);
	for n=1:nbSamples
		plot(1:nbData, w(k,(n-1)*nbData+1:n*nbData), '-','linewidth',2,'color',[.7 .7 .7]);
	end
	axis([1 nbData 0 1.05]); set(gca,'Xtick',[]); set(gca,'Ytick',[]);
	xlabel('$t$','fontsize',18,'interpreter','latex'); 
	ylabel(['$\dot{x}_' num2str(k) '$'],'fontsize',18,'interpreter','latex');
end

norm(hmix-w)

%print('-dpng','graphs/demo_profileGMM_multivariate01.png');
pause;
close all;
