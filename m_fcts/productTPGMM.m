function [Mu, Sigma] = productTPGMM(model, p)
% Sylvain Calinon, Leonel Rozo, 2014
%
% Compute the product of Gaussians for a task-parametrized model where the
% set of parameters are stored in the variable 'p'.

% TP-GMM products
%See Eq. (6.0.5), (6.0.6) and (6.0.7) in doc/TechnicalReport.pdf
for i = 1:model.nbStates
	% Reallocating
	SigmaTmp = zeros(model.nbVar);
	MuTmp = zeros(model.nbVar,1);
	% Product of Gaussians
	for m = 1 : model.nbFrames
		MuP = p(m).A * model.Mu(:,m,i) + p(m).b;
		SigmaP = p(m).A * model.Sigma(:,:,m,i) * p(m).A';
		SigmaTmp = SigmaTmp + inv(SigmaP);
		MuTmp = MuTmp + SigmaP\MuP;
	end
	Sigma(:,:,i) = inv(SigmaTmp);
	Mu(:,i) = Sigma(:,:,i) * MuTmp;
end
