function [Mu, Sigma] = productTPGMM0(model, p)
%Compute the product of Gaussians for a TP-GMM, where the set of task parameters are stored in the variable 'p'.
%Sylvain Calinon, 2015

%TP-GMM products, see Eqs (6.0.5)-(6.0.7) in doc/TechnicalReport.pdf
for i=1:model.nbStates
	% Reallocating
	SigmaTmp = zeros(model.nbVar);
	MuTmp = zeros(model.nbVar,1);
	% Product of Gaussians
	for m=1:model.nbFrames
		MuP = p(m).A * model.Mu(:,m,i) + p(m).b;
		SigmaP = p(m).A * model.Sigma(:,:,m,i) * p(m).A';
		SigmaTmp = SigmaTmp + inv(SigmaP);
		MuTmp = MuTmp + SigmaP\MuP;
	end
	Sigma(:,:,i) = inv(SigmaTmp);
	Mu(:,i) = Sigma(:,:,i) * MuTmp;
end
