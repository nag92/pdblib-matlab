function [Mu, Sigma] = productTPGMM(model, p)
%Compute the product of Gaussians for a task-parametrized model where the
%set of parameters are stored in the variable 'p'.
%Sylvain Calinon, 2015

%diagRegularizationFactor = 1E-6;
diagRegularizationFactor = 1E-4;

%TP-GMM products
for i=1:model.nbStates
	%Reallocating
	SigmaTmp = zeros(model.nbVar);
	MuTmp = zeros(model.nbVar,1);
	%Product of Gaussians
	for m=1:model.nbFrames
		MuP = p(m).A * model.Mu(1:model.nbVars(m),m,i) + p(m).b;
		SigmaP = p(m).A * model.Sigma(1:model.nbVars(m),1:model.nbVars(m),m,i) * p(m).A' + eye(model.nbVar)*diagRegularizationFactor;
		SigmaTmp = SigmaTmp + inv(SigmaP);
		MuTmp = MuTmp + SigmaP\MuP;
	end
	Sigma(:,:,i) = inv(SigmaTmp);
	Mu(:,i) = Sigma(:,:,i) * MuTmp;
end
