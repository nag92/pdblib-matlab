function model = EM_tensorMFA(Data, model)
%Training of a task-parameterized mixture of factor analyzers (TP-MFA) with an expectation-maximization (EM) algorithm.
%Sylvain Calinon, 2015

%Parameters of the EM algorithm
nbMinSteps = 5; %Minimum number of iterations allowed
nbMaxSteps = 100; %Maximum number of iterations allowed
maxDiffLL = 1E-5; %Likelihood increase threshold to stop the algorithm
nbData = size(Data,3);

%diagRegularizationFactor = 1E-2; %Regularization term is optional, see Eq. (2.1.2) in doc/TechnicalReport.pdf
diagRegularizationFactor = 1E-10; %Regularization term is optional, see Eq. (2.1.2) in doc/TechnicalReport.pdf

% %Initialization of the MFA parameters
% Itmp = eye(model.nbVar)*1E-2;
% model.P = repmat(Itmp, [1 1 model.nbFrames model.nbStates]);
% model.L = repmat(Itmp(:,1:model.nbFA), [1 1 model.nbFrames model.nbStates]);

%Initialization of the MFA parameters
for i=1:model.nbStates
	for m=1:model.nbFrames
		model.P(:,:,m,i) = diag(diag(model.Sigma(:,:,m,i)));
		[V,D] = eig(model.Sigma(:,:,m,i)-model.P(:,:,m,i)); 
		[~,id] = sort(diag(D),'descend');
		V = V(:,id)*D(id,id).^.5;
		model.L(:,:,m,i) = V(:,1:model.nbFA);
	end
end
for nbIter=1:nbMaxSteps
	for i=1:model.nbStates
		for m=1:model.nbFrames
			%Update B,L,P
			B(:,:,m,i) = model.L(:,:,m,i)' / (model.L(:,:,m,i) * model.L(:,:,m,i)' + model.P(:,:,m,i));
			model.L(:,:,m,i) = model.Sigma(:,:,m,i) * B(:,:,m,i)' / (eye(model.nbFA) - B(:,:,m,i) * model.L(:,:,m,i) + B(:,:,m,i) * model.Sigma(:,:,m,i) * B(:,:,m,i)');
			model.P(:,:,m,i) = diag(diag(model.Sigma(:,:,m,i) - model.L(:,:,m,i) * B(:,:,m,i) * model.Sigma(:,:,m,i)));
		end
	end
end

%EM loop
for nbIter=1:nbMaxSteps
	fprintf('.');
	
	%E-step
	[Lik, GAMMA] = computeGamma(Data, model); %See 'computeGamma' function below
	GAMMA2 = GAMMA ./ repmat(sum(GAMMA,2),1,nbData);
	model.Pix = GAMMA2;
	
	%M-step
	for i=1:model.nbStates
		
		%Update Priors, see Eq. (6.0.2) in doc/TechnicalReport.pdf
		model.Priors(i) = sum(sum(GAMMA(i,:))) / nbData;
		
		for m=1:model.nbFrames
			%Matricization/flattening of tensor
			DataMat(:,:) = Data(:,m,:);
			
			%Update Mu, see Eq. (6.0.3) in doc/TechnicalReport.pdf
			model.Mu(:,m,i) = DataMat * GAMMA2(i,:)';
			
			%Compute covariance, see Eq. (2.2.15) in doc/TechnicalReport.pdf
			DataTmp = DataMat - repmat(model.Mu(:,m,i),1,nbData);
			S(:,:,m,i) = DataTmp * diag(GAMMA2(i,:)) * DataTmp' + eye(model.nbVar)*diagRegularizationFactor;
			
			%Update B, see Eq. (2.2.16) in doc/TechnicalReport.pdf
			B(:,:,m,i) = model.L(:,:,m,i)' / (model.L(:,:,m,i) * model.L(:,:,m,i)' + model.P(:,:,m,i));
			%Update Lambda, see Eq. (2.2.12) in doc/TechnicalReport.pdf
			model.L(:,:,m,i) = S(:,:,m,i) * B(:,:,m,i)' / (eye(model.nbFA) - B(:,:,m,i) * model.L(:,:,m,i) + B(:,:,m,i) * S(:,:,m,i) * B(:,:,m,i)');
			%Update Psi, see Eq. (2.2.13) in doc/TechnicalReport.pdf
			model.P(:,:,m,i) = diag(diag(S(:,:,m,i) - model.L(:,:,m,i) * B(:,:,m,i) * S(:,:,m,i))) + eye(model.nbVar)*diagRegularizationFactor;

			%Reconstruct Sigma, see Eqs (2.2.4) and (6.0.4) in doc/TechnicalReport.pdf
			model.Sigma(:,:,m,i) = model.L(:,:,m,i) * model.L(:,:,m,i)' + model.P(:,:,m,i);
		end
	end
	
	%Compute average log-likelihood
	LL(nbIter) = sum(log(sum(Lik,1))) / size(Lik,2);
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
function [Lik, GAMMA, GAMMA0] = computeGamma(Data, model)
%See Eq. (6.0.1) in doc/TechnicalReport.pdf
nbData = size(Data, 3);
Lik = ones(model.nbStates, nbData);
GAMMA0 = zeros(model.nbStates, model.nbFrames, nbData);
for i=1:model.nbStates
	for m=1:model.nbFrames
		DataMat(:,:) = Data(:,m,:); %Matricization/flattening of tensor
		GAMMA0(i,m,:) = gaussPDF(DataMat, model.Mu(:,m,i), model.Sigma(:,:,m,i));
		Lik(i,:) = Lik(i,:) .* squeeze(GAMMA0(i,m,:))';
	end
	Lik(i,:) = Lik(i,:) * model.Priors(i);
end
GAMMA = Lik ./ repmat(sum(Lik,1)+realmin, size(Lik,1), 1);
end




