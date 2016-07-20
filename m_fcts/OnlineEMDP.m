function [ model,N ] = OnlineEMDP( N,P,MinSigma,model,lambda )
%ONLINEEMDP Online clustering with DP Means
%This function updates a Gaussian Mixture Model by using an extended version of DPMEANS algorithm.
%
%Author: Danilo Bruno, 2015
%
%Refs:
%for DPMEANS :
%Kulis, B. and Jordan, M. I. (2012). Revisiting k-means: New algorithms via bayesian nonparametrics. In Proc. Intl Conf. on Machine Learning (ICML), Edimburgh (UK).
%
%for MAP update of GMM:
%Gauvain, J.-L. and Lee, C.-H. (1994). Maximum a posteriori estimation for multivariate gaussian mixture observations of markov chians. IEE Transactions on Speech and Audio Processing, 2(2):291–298.
%
%
%Variables
%N: 		current number of points processed
%P: 		current point
%MinSigma :	minimum covariance for regularization
%model :	GMM model containing
%				model.Mu :		Mean
%				model.Sigma :	Covariance
%				model.Priors :	Priors
%lambda :	splitting distance

if N == 0 		%if no point exists, create gmm with current point
	nbVar = size(P,1);
	Mu = P;
	Sigma = MinSigma*eye(nbVar);
	Priors = 1;
	N = 1;
	sp = 0.0001;
else
	Mu = model.Mu;
	Sigma = model.Sigma;
	Priors = model.Priors;

	nbVar = size(Mu,1);
	nbStates = size(Sigma,3);
	N = N+1;

	% Evaluate distance of current point from current components
	d = zeros(1,nbStates+1);
	for k=1:nbStates
		d(k) = sqrt((P-Mu(:,k))'*(P-Mu(:,k)));
	end
	% set splitting distance as additional cluster
	d(nbStates+1) = lambda;
	[~,idx] = min(d);
	if idx == nbStates + 1	%If more distant than lambda, create new cluster
		nbStates = nbStates + 1;
		Mu(:,nbStates) = P;
		Sigma(:,:,nbStates) = MinSigma*eye(nbVar);
		Priors(nbStates) = 1/N;
	else					%Update closest cluster with MAP estimate
		PriorsTmp = 1/N + Priors(idx);
		MuTmp = 1/PriorsTmp*(Priors(idx)*Mu(:,idx)+P/N);
		Sigma(:,:,idx) = Priors(idx)/PriorsTmp*(Sigma(:,:,idx)+(Mu(:,idx)-MuTmp)*(Mu(:,idx)-MuTmp)') + 1/(N*PriorsTmp)*(MinSigma*eye(nbVar)+(P-MuTmp)*(P-MuTmp)');
		Mu(:,idx) = MuTmp;
		Priors(idx) = PriorsTmp;
	end
	Priors = Priors/sum(Priors);
end

model.Mu = Mu;
model.Sigma = Sigma;
model.Priors = Priors;
	
end

