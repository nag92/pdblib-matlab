function demo_Gaussian06
% Gaussian reformulated as zero-centered Gaussian with augmented covariance
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
nbVar = 4; %Dimension of datapoint
nbData = 10; %Number of datapoints


%% Gaussian with augmented covariance
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x = rand(nbVar,nbData);
Mu = mean(x,2);
Sigma = cov(x');
% Sigma = eye(nbVar);
Sigma2 = [Sigma+Mu*Mu', Mu; Mu', 1]; % .* (det(Sigma).^(-1./(nbVar+1)));

h = gaussPDF(x, Mu, Sigma);
h = h / sum(h);

h2 = gaussPDF([x; ones(1,nbData)], zeros(nbVar+1,1), Sigma2);
h2 = h2 / sum(h2);


%% Plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure; hold on;
plot(h,'k.','markersize',20);
plot(h2,'ro','markersize',10,'linewidth',2);
xlabel('x'); ylabel('p(x)');

pause;
close all;
