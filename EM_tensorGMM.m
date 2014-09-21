function model = EM_tensorGMM(Data, model)
% Training of a task-parameterized Gaussian mixture model (GMM) with an expectation-maximization (EM) algorithm. 
% The approach allows the modulation of the centers and covariance matrices of the Gaussians with respect to 
% external parameters represented in the form of candidate coordinate systems.   
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

%Parameters of the EM algorithm
nbMinSteps = 5; %Minimum number of iterations allowed
nbMaxSteps = 100; %Maximum number of iterations allowed
maxDiffLL = 1E-4; %Likelihood increase threshold to stop the algorithm
nbData = size(Data,3);

%diagRegularizationFactor = 1E-2;
diagRegularizationFactor = 1E-4;

for nbIter=1:nbMaxSteps
	fprintf('.');
	%E-step
	[L, GAMMA, GAMMA0] = computeGamma(Data, model); %See 'computeGamma' function below and Eq. (2.0.5) in doc/TechnicalReport.pdf
	GAMMA2 = GAMMA ./ repmat(sum(GAMMA,2),1,nbData);
	%M-step
	for i=1:model.nbStates 
		%Update Priors
		model.Priors(i) = sum(sum(GAMMA(i,:))) / nbData; %See Eq. (2.0.6) in doc/TechnicalReport.pdf
		for m=1:model.nbFrames
			%Matricization/flattening of tensor
			DataMat(:,:) = Data(:,m,:);
			%Update Mu 
			model.Mu(:,m,i) = DataMat * GAMMA2(i,:)'; %See Eq. (2.0.7) in doc/TechnicalReport.pdf
			%Update Sigma (regularization term is optional) 
			DataTmp = DataMat - repmat(model.Mu(:,m,i),1,nbData);
			model.Sigma(:,:,m,i) = DataTmp * diag(GAMMA2(i,:)) * DataTmp' + eye(model.nbVar) * diagRegularizationFactor; %See Eq. (2.0.8) and (2.1.2) in doc/TechnicalReport.pdf
		end
	end
	%Compute average log-likelihood 
	LL(nbIter) = sum(log(sum(L,1))) / size(L,2); %See Eq. (2.0.4) in doc/TechnicalReport.pdf
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
function [L, GAMMA, GAMMA0] = computeGamma(Data, model)
	%See Eq. (2.0.5) in doc/TechnicalReport.pdf
	nbData = size(Data, 3);
	L = ones(model.nbStates, nbData);
	GAMMA0 = zeros(model.nbStates, model.nbFrames, nbData);
	for m=1:model.nbFrames
		DataMat(:,:) = Data(:,m,:); %Matricization/flattening of tensor
		for i=1:model.nbStates
			GAMMA0(i,m,:) = model.Priors(i) * gaussPDF(DataMat, model.Mu(:,m,i), model.Sigma(:,:,m,i));
			L(i,:) = L(i,:) .* squeeze(GAMMA0(i,m,:))'; 
		end
	end
	%Normalization
	GAMMA = L ./ repmat(sum(L,1)+realmin,size(L,1),1);
end




function model = EM_tensorGMM(Data, model)
% Training of a task-parameterized Gaussian mixture model (GMM) with an expectation-maximization (EM) algorithm. 
% The approach allows the modulation of the centers and covariance matrices of the Gaussians with respect to 
% external parameters represented in the form of candidate coordinate systems.   
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

%Parameters of the EM algorithm
nbMinSteps = 5; %Minimum number of iterations allowed
nbMaxSteps = 100; %Maximum number of iterations allowed
maxDiffLL = 1E-4; %Likelihood increase threshold to stop the algorithm
nbData = size(Data,3);

%diagRegularizationFactor = 1E-2;
diagRegularizationFactor = 1E-4;

for nbIter=1:nbMaxSteps
  fprintf('.');
  %E-step
  [L, GAMMA, GAMMA0] = computeGamma(Data, model); %See 'computeGamma' function below and Eq. (2.0.5) in doc/TechnicalReport.pdf
  GAMMA2 = GAMMA ./ repmat(sum(GAMMA,2),1,nbData);
  %M-step
  for i=1:model.nbStates 
    %Update Priors
    model.Priors(i) = sum(sum(GAMMA(i,:))) / nbData; %See Eq. (2.0.6) in doc/TechnicalReport.pdf
    for m=1:model.nbFrames
      %Matricization/flattening of tensor
      DataMat(:,:) = Data(:,m,:);
      %Update Mu 
      model.Mu(:,m,i) = DataMat * GAMMA2(i,:)'; %See Eq. (2.0.7) in doc/TechnicalReport.pdf
      %Update Sigma (regularization term is optional) 
      DataTmp = DataMat - repmat(model.Mu(:,m,i),1,nbData);
      model.Sigma(:,:,m,i) = DataTmp * diag(GAMMA2(i,:)) * DataTmp' + eye(model.nbVar) * diagRegularizationFactor; %See Eq. (2.0.8) and (2.1.2) in doc/TechnicalReport.pdf
    end
  end
  %Compute average log-likelihood 
  LL(nbIter) = sum(log(sum(L,1))) / size(L,2); %See Eq. (2.0.4) in doc/TechnicalReport.pdf
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
function [L, GAMMA, GAMMA0] = computeGamma(Data, model)
  %See Eq. (2.0.5) in doc/TechnicalReport.pdf
  nbData = size(Data, 3);
  L = ones(model.nbStates, nbData);
  GAMMA0 = zeros(model.nbStates, model.nbFrames, nbData);
  for m=1:model.nbFrames
    DataMat(:,:) = Data(:,m,:); %Matricization/flattening of tensor
    for i=1:model.nbStates
      GAMMA0(i,m,:) = model.Priors(i) * gaussPDF(DataMat, model.Mu(:,m,i), model.Sigma(:,:,m,i));
      L(i,:) = L(i,:) .* squeeze(GAMMA0(i,m,:))'; 
    end
  end
  %Normalization
  GAMMA = L ./ repmat(sum(L,1)+realmin,size(L,1),1);
end




