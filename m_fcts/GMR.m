function [expData, expSigma, H] = GMR(model, DataIn, in, out)
% Gaussian mixture regression (GMR)
%
% Authors:	Sylvain Calinon, Danilo Bruno, 2014
%         	http://programming-by-demonstration.org/lib/
%
% This source code is given for free! In exchange, we would be grateful if you cite
% the following reference in any academic publication that uses this code or part of it:
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

nbData = size(DataIn,2);
nbVarOut = length(out);

diagRegularizationFactor = 1E-8;

MuTmp = zeros(nbVarOut,model.nbStates);
expData = zeros(nbVarOut,nbData);
expSigma = zeros(nbVarOut,nbVarOut,nbData);
for t=1:nbData
	%Compute activation weight
	%See Eq. (3.0.5) in doc/TechnicalReport.pdf
	for i=1:model.nbStates
		H(i,t) = model.Priors(i) * gaussPDF(DataIn(:,t), model.Mu(in,i), model.Sigma(in,in,i));
	end
	H(:,t) = H(:,t)/sum(H(:,t)+realmin);
	%Compute expected conditional means
	%See Eq. (3.0.8) in doc/TechnicalReport.pdf
	for i=1:model.nbStates
		MuTmp(:,i) = model.Mu(out,i) + model.Sigma(out,in,i)/model.Sigma(in,in,i) * (DataIn(:,t)-model.Mu(in,i));
		expData(:,t) = expData(:,t) + H(i,t) * MuTmp(:,i);
	end
	%Compute expected conditional covariances
	%See Eq. (3.0.14) in doc/TechnicalReport.pdf
	for i=1:model.nbStates
		SigmaTmp = model.Sigma(out,out,i) - model.Sigma(out,in,i)/model.Sigma(in,in,i) * model.Sigma(in,out,i);
		expSigma(:,:,t) = expSigma(:,:,t) + H(i,t) * (SigmaTmp + MuTmp(:,i)*MuTmp(:,i)');
	end
	expSigma(:,:,t) = expSigma(:,:,t) - expData(:,t)*expData(:,t)' + eye(nbVarOut) * diagRegularizationFactor; 
end

