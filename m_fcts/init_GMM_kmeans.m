function model = init_GMM_kmeans(Data, model)
% This function initializes the parameters of a Gaussian Mixture Model 
% (GMM) by using k-means clustering algorithm.
%
% Inputs -----------------------------------------------------------------
%   o Data:     D x N array representing N datapoints of D dimensions.
%   o nbStates: Number K of GMM components.
% Outputs ----------------------------------------------------------------
%   o Priors:   1 x K array representing the prior probabilities of the
%               K GMM components.
%   o Mu:       D x K array representing the centers of the K GMM components.
%   o Sigma:    D x D x K array representing the covariance matrices of the 
%               K GMM components.
%
% Author:	Sylvain Calinon, 2014
%         http://programming-by-demonstration.org/lib/

nbVar = size(Data,1);
diagRegularizationFactor = 1E-2;

[Data_id, model.Mu] = kmeansClustering(Data, model.nbStates);

for i=1:model.nbStates
  idtmp = find(Data_id==i);
  model.Priors(i) = length(idtmp);
  model.Sigma(:,:,i) = cov([Data(:,idtmp) Data(:,idtmp)]');
  %Regularization term to avoid numerical instability
  model.Sigma(:,:,i) = model.Sigma(:,:,i) + eye(nbVar)*diagRegularizationFactor;
end
model.Priors = model.Priors / sum(model.Priors);
