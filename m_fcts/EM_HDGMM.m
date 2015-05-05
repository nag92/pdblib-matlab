function [model, GAMMA2] = EM_HDGMM(Data, model)
%EM for High Dimensional Data Clustering (HDDC, HD-GMM) model proposed by Bouveyron (2007) 
%Sylvain Calinon, 2015

%Parameters of the EM iterations
nbMinSteps = 5; %Minimum number of iterations allowed
nbMaxSteps = 100; %Maximum number of iterations allowed
maxDiffLL = 1E-4; %Likelihood increase threshold to stop the algorithm
nbData = size(Data,2);

diagRegularizationFactor = 1E-8; %Regularization term is optional, see Eq. (2.1.2) in doc/TechnicalReport.pdf

%EM loop
for nbIter=1:nbMaxSteps
	fprintf('.');
	
	%E-step
	[Lik, GAMMA] = computeGamma(Data, model); %See 'computeGamma' function below
	GAMMA2 = GAMMA ./ repmat(sum(GAMMA,2), 1, nbData);
	
	%M-step
	%Update Priors, see Eq. (2.0.6) in doc/TechnicalReport.pdf
	model.Priors = sum(GAMMA,2)/nbData;
	
	%Update Mu, see Eq. (2.0.7) in doc/TechnicalReport.pdf
	model.Mu = Data * GAMMA2';
	
	%Update factor analyser params
	for i=1:model.nbStates
		%Compute covariance
		DataTmp = Data - repmat(model.Mu(:,i),1,nbData);
		S(:,:,i) = DataTmp * diag(GAMMA2(i,:)) * DataTmp' + eye(model.nbVar) * diagRegularizationFactor;

		%HDGMM update, see Eq. (2.2.2) in doc/TechnicalReport.pdf
		[V,D] = eig(S(:,:,i)); 
		[~,id] = sort(diag(D),'descend');
% 		model.D(:,:,i) = D(id(1:model.nbFA), id(1:model.nbFA));
% 		model.V(:,:,i) = V(:, id(1:model.nbFA)); 
		d = diag(D);
		model.D(:,:,i) = diag([d(id(1:model.nbFA)); repmat(mean(d(id(model.nbFA+1:end))), model.nbVar-model.nbFA, 1)]);
		model.V(:,:,i) = V(:,id); 
	
		%Reconstruct Sigma, see Eq. (2.2.1) in doc/TechnicalReport.pdf
		model.Sigma(:,:,i) = model.V(:,:,i) * model.D(:,:,i) * model.V(:,:,i)' + eye(model.nbVar) * diagRegularizationFactor;
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
