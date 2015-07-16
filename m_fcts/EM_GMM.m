function [model, GAMMA2, LL] = EM_GMM(Data, model)
% Training of a Gaussian mixture model (GMM) with an expectation-maximization (EM) algorithm.
%
% Author:	Sylvain Calinon, 2014
%         http://programming-by-demonstration.org/SylvainCalinon

%Parameters of the EM algorithm
nbMinSteps = 5; %Minimum number of iterations allowed
nbMaxSteps = 100; %Maximum number of iterations allowed
maxDiffLL = 1E-4; %Likelihood increase threshold to stop the algorithm
nbData = size(Data,2);

%diagRegularizationFactor = 1E-6; %Regularization term is optional, see Eq. (2.1.2) in doc/TechnicalReport.pdf
diagRegularizationFactor = 1E-4; %Regularization term is optional, see Eq. (2.1.2) in doc/TechnicalReport.pdf

for nbIter=1:nbMaxSteps
	fprintf('.');
	
	%E-step
	[L, GAMMA] = computeGamma(Data, model); %See 'computeGamma' function below
	GAMMA2 = GAMMA ./ repmat(sum(GAMMA,2),1,nbData);
	
	%M-step
	for i=1:model.nbStates
		%Update Priors, see Eq. (2.0.6) in doc/TechnicalReport.pdf
		model.Priors(i) = sum(GAMMA(i,:)) / nbData;
		
		%Update Mu, see Eq. (2.0.7) in doc/TechnicalReport.pdf
		model.Mu(:,i) = Data * GAMMA2(i,:)';
		
		%Update Sigma, see Eq. (2.0.8) in doc/TechnicalReport.pdf (regularization term is optional, see Eq. (2.1.2))
		DataTmp = Data - repmat(model.Mu(:,i),1,nbData);
		model.Sigma(:,:,i) = DataTmp * diag(GAMMA2(i,:)) * DataTmp' + eye(size(Data,1)) * diagRegularizationFactor;
	end
	
	%Compute average log-likelihood
	LL(nbIter) = sum(log(sum(L,1))) / nbData;
	%Stop the algorithm if EM converged (small change of LL)
	if nbIter>nbMinSteps
		if LL(nbIter)-LL(nbIter-1)<maxDiffLL || nbIter==nbMaxSteps-1
			disp(['EM converged after ' num2str(nbIter) ' iterations.']);
			return;
		end
	end
end
disp(['The maximum number of ' num2str(nbMaxSteps) ' EM iterations has been reached.']);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [L, GAMMA] = computeGamma(Data, model)
%See Eq. (2.0.5) in doc/TechnicalReport.pdf
L = zeros(model.nbStates,size(Data,2));
for i=1:model.nbStates
	L(i,:) = model.Priors(i) * gaussPDF(Data, model.Mu(:,i), model.Sigma(:,:,i));
end
GAMMA = L ./ repmat(sum(L,1)+realmin, model.nbStates, 1);
end



