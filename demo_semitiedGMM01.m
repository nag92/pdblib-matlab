function demo_semitiedGMM01
% Semi-tied Gaussian Mixture Model by tying the covariance matrices of a
% Gaussian mixture model with a set of common basis vectors.
%
% Writing code takes time. Polishing it and making it available to others takes longer!
% If some parts of the code were useful for your research of for a better understanding
% of the algorithms, please reward the authors by citing the related publications,
% and consider making your own research available in this way.
%
% @article{Tanwani16RAL,
%   author="Tanwani, A. K. and Calinon, S.",
%   title="Learning Robot Manipulation Tasks with Task-Parameterized Semi-Tied Hidden Semi-{M}arkov Model",
%   journal="{IEEE} Robotics and Automation Letters ({RA-L})",
%   year="2016",
%   month="January",
%   volume="1",
%   number="1",
%   pages="235--242",
% 	doi="10.1109/LRA.2016.2517825"
% }
%
% Copyright (c) 2015 Idiap Research Institute, http://idiap.ch/
% Written by Ajay Tanwani and Sylvain Calinon
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


%% Dataset and parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('data/Zshape3D.mat');
model.nbVar = size(Data,1);
model.nbStates = 3;
model.time_dim = false;
model.nbSamples = 1;
%Algorithm parameters
model.params_alpha = 1.0;
model.params_Bsf = 5E-2;


%% Learning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model = init_GMM_timeBased(Data, model);
if isfield(model,'time_dim')
	if ~model.time_dim
		model.Mu = model.Mu(2:model.nbVar,:);
		model.Sigma = model.Sigma(2:model.nbVar,2:model.nbVar,:);
		Data = Data(2:model.nbVar,:);
		model.nbVar= model.nbVar - 1;
	end
end
model = EM_semitiedGMM(Data, model);


%% Plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('color',[1 1 1],'Position',[10 10 800 650]); hold on; axis off; box off;
xx = round(linspace(1,64,9)); xx2 = round(linspace(1,64,3));
clrmap = colormap('jet'); clrmap = min(clrmap(xx,:),.95);
clrmap2 = colormap('jet'); clrmap2 = min(clrmap2(xx2,:),.95);
for i=1:5
	plot3(Data(1,(i-1)*300+1:300*i), Data(2,(i-1)*300 + 1:300*i),Data(3,(i-1)*300+1:300*i),'-','linewidth',1.5,'color',[.5 .5 .5]);
end
clrlist1 = [2,3,1];
for i=1:model.nbVar
	mArrow3(zeros(model.nbVar,1),model.H(:,i),'color',clrmap2(clrlist1(i),:),'stemWidth',0.75, 'tipWidth',1.0, 'facealpha',0.75);
end
clrlist = [2,7,4];
for i=1:model.nbStates
	plotGMM3D(model.Mu(:,i), model.Sigma(:,:,i), clrmap(clrlist(i),:), .5);
	for j=1:model.nbVar
		mArrow3(model.Mu(:,i), model.Mu(:,i) + model.H(:,j).*(model.SigmaDiag(j,j,i).^0.5),'color',clrmap2(clrlist1(j),:),'stemWidth',0.75, 'tipWidth',1.25, 'facealpha',1);
	end
end
view(-40,6); axis equal;

%print('-dpng','graphs/demo_semitiedGMM01.png');
pause;
close all;

