function r = estimateAttractorPath(DataIn, model, r)
% Estimation of an attractor path from a task-parameterized GMM and a set of candidate frames.
%
% Author:	Sylvain Calinon, 2014
%         http://programming-by-demonstration.org/SylvainCalinon
%
% This source code is given for free! In exchange, I would be grateful if you cite  
% the following reference in any academic publication that uses this code or part of it: 
%
% @inproceedings{Calinon14ICRA,
%   author="Calinon, S. and Bruno, D. and Caldwell, D. G.",
%   title="A task-parameterized probabilistic model with minimal intervention control",
%   booktitle="Proc. {IEEE} Intl Conf. on Robotics and Automation ({ICRA})",
%   year="2014",
%   month="May-June",
%   address="Hong Kong, China",
%   pages="3339--3344"
% }

in = 1:size(DataIn,1);
out = in(end)+1:model.nbVar;

%% Estimation of the attractor path by Gaussian mixture regression,
%% by using the GMM resulting from the product of linearly transformed Gaussians
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[r.Mu, r.Sigma] = productTPGMM(model, r.p);
r.Priors = model.Priors;  
r.nbStates = model.nbStates;

[r.currTar, r.currSigma] = GMR(r, DataIn, in, out);



