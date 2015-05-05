function [model, GAMMA2] = EM_MPPCA(Data, model)
%EM for mixture of probabilistic principal component analyzers,
%inspired by "Mixtures of Probabilistic Principal Component Analysers" by Michael E. Tipping and Christopher M. Bishop
%Sylvain Calinon, 2015

%Parameters of the EM iterations
nbMinSteps = 5; %Minimum number of iterations allowed
nbMaxSteps = 100; %Maximum number of iterations allowed
maxDiffLL = 1E-4; %Likelihood increase threshold to stop the algorithm
nbData = size(Data,2);

diagRegularizationFactor = 1E-6; %Regularization term is optional, see Eq. (2.1.2) in doc/TechnicalReport.pdf

%Initialization of the MPPCA parameters from eigendecomposition
for i=1:model.nbStates
	model.o(i) = trace(model.Sigma(:,:,i)) / model.nbVar;
	[V,D] = eig(model.Sigma(:,:,i)-eye(model.nbVar)*model.o(i)); 
	[~,id] = sort(diag(D),'descend');
	V = V(:,id)*D(id,id).^.5;
	model.L(:,:,i) = V(:,1:model.nbFA);
end

%EM loop
for nbIter=1:nbMaxSteps
	fprintf('.');
	
	%E-step
	[Lik, GAMMA] = computeGamma(Data, model); %See 'computeGamma' function below
	GAMMA2 = GAMMA ./ repmat(sum(GAMMA,2),1,nbData);
	
	%M-step
	%Update Priors, see Eq. (2.2.10) in doc/TechnicalReport.pdf
	model.Priors = sum(GAMMA,2) / nbData;
	
	%Update Mu, see Eq. (2.2.11) in doc/TechnicalReport.pdf
	model.Mu = Data * GAMMA2';
	
	%Update factor analyser params
	for i=1:model.nbStates
		%Compute covariance, see Eq. (2.2.19) in doc/TechnicalReport.pdf
		DataTmp = Data - repmat(model.Mu(:,i),1,nbData);
		S(:,:,i) = DataTmp * diag(GAMMA2(i,:)) * DataTmp' + eye(model.nbVar) * diagRegularizationFactor;

		%Update M, see Eq. (2.2.20) in doc/TechnicalReport.pdf 
		M = eye(model.nbFA)*model.o(i) + model.L(:,:,i)' * model.L(:,:,i);
		%Update Lambda, see Eq. (2.2.17) in doc/TechnicalReport.pdf 
		Lnew =  S(:,:,i) * model.L(:,:,i) / (eye(model.nbFA)*model.o(i) + M \ model.L(:,:,i)' * S(:,:,i) * model.L(:,:,i));
		%Update of sigma^2, see Eq. (2.2.21) in doc/TechnicalReport.pdf 
		model.o(i) = trace(S(:,:,i) - S(:,:,i) * model.L(:,:,i) / M * Lnew') / model.nbVar;
		model.L(:,:,i) = Lnew;
		%Update Psi, see Eq. (2.2.18) in doc/TechnicalReport.pdf 
		model.P(:,:,i) = eye(model.nbVar) * model.o(i);
		
		%Reconstruct Sigma, see Eq. (2.2.4) in doc/TechnicalReport.pdf
		model.Sigma(:,:,i) = model.L(:,:,i) * model.L(:,:,i)' + model.P(:,:,i);
	end
	
	%Compute average log-likelihood
	LL(nbIter) = sum(log(sum(Lik,1))) / nbData;
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
function [Lik, GAMMA] = computeGamma(Data, model)
%See Eq. (2.0.5) in doc/TechnicalReport.pdf
Lik = zeros(model.nbStates,size(Data,2));
for i=1:model.nbStates
	Lik(i,:) = model.Priors(i) * gaussPDF(Data, model.Mu(:,i), model.Sigma(:,:,i));
end
GAMMA = Lik ./ repmat(sum(Lik,1)+realmin, model.nbStates, 1);
end
