function [model, GAMMA2] = EM_MFA(Data, model)
%EM for Mixture of factor analysis
%Implementation inspired by "Parsimonious Gaussian Mixture Models" by McNicholas and Murphy, Appendix 8, p.17 (UUU version)
%Sylvain Calinon, 2015

%Parameters of the EM iterations
nbMinSteps = 5; %Minimum number of iterations allowed
nbMaxSteps = 100; %Maximum number of iterations allowed
maxDiffLL = 1E-4; %Likelihood increase threshold to stop the algorithm
nbData = size(Data,2);

diagRegularizationFactor = 1E-6; %Regularization term is optional, see Eq. (2.1.2) in doc/TechnicalReport.pdf

% %Circular initialization of the MFA parameters
% Itmp = eye(model.nbVar)*1E-2;
% model.P = repmat(Itmp, [1 1 model.nbStates]);
% model.L = repmat(Itmp(:,1:model.nbFA), [1 1 model.nbStates]);

%Initialization of the MFA parameters from eigendecomposition estimate
for i=1:model.nbStates
	model.P(:,:,i) = diag(diag(model.Sigma(:,:,i))); %Dimension-wise variance
	[V,D] = eig(model.Sigma(:,:,i)-model.P(:,:,i)); 
	[~,id] = sort(diag(D),'descend');
	V = V(:,id)*D(id,id).^.5;
	model.L(:,:,i) = V(:,1:model.nbFA); %->Sigma=LL'+P
end
for nbIter=1:nbMaxSteps
	for i=1:model.nbStates
		%Update B,L,P
		B(:,:,i) = model.L(:,:,i)' / (model.L(:,:,i) * model.L(:,:,i)' + model.P(:,:,i));
		model.L(:,:,i) = model.Sigma(:,:,i) * B(:,:,i)' / (eye(model.nbFA) - B(:,:,i) * model.L(:,:,i) + B(:,:,i) * model.Sigma(:,:,i) * B(:,:,i)');
		model.P(:,:,i) = diag(diag(model.Sigma(:,:,i) - model.L(:,:,i) * B(:,:,i) * model.Sigma(:,:,i)));
	end
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
	
	%Update factor analysers parameters
	for i=1:model.nbStates
		%Compute covariance, see Eq. (2.2.15) in doc/TechnicalReport.pdf
		DataTmp = Data - repmat(model.Mu(:,i),1,nbData);
		S(:,:,i) = DataTmp * diag(GAMMA2(i,:)) * DataTmp' + eye(model.nbVar) * diagRegularizationFactor;

		%Update B, see Eq. (2.2.16) in doc/TechnicalReport.pdf
		B(:,:,i) = model.L(:,:,i)' / (model.L(:,:,i) * model.L(:,:,i)' + model.P(:,:,i));
		%Update Lambda, see Eq. (2.2.12) in doc/TechnicalReport.pdf
		model.L(:,:,i) = S(:,:,i) * B(:,:,i)' / (eye(model.nbFA) - B(:,:,i) * model.L(:,:,i) + B(:,:,i) * S(:,:,i) * B(:,:,i)');
		%Update Psi, see Eq. (2.2.13) in doc/TechnicalReport.pdf
		model.P(:,:,i) = diag(diag(S(:,:,i) - model.L(:,:,i) * B(:,:,i) * S(:,:,i))) + eye(model.nbVar) * diagRegularizationFactor;

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
%See Eq. (2.2.9) in doc/TechnicalReport.pdf
Lik = zeros(model.nbStates,size(Data,2));
for i=1:model.nbStates
	Lik(i,:) = model.Priors(i) * gaussPDF(Data, model.Mu(:,i), model.Sigma(:,:,i));
end
GAMMA = Lik ./ repmat(sum(Lik,1)+realmin, model.nbStates, 1);
end
