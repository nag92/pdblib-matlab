function model = init_tensorGMM_kmeans(Data, model)
% Author:   Leonel Rozo, 2014
%           http://programming-by-demonstration.org/LeonelRozo
%

diagRegularizationFactor = 1E-4;

%Matricization/flattening of tensor
DataAll = reshape(Data, size(Data,1)*size(Data,2), size(Data,3)); 

%The function 'kmeans' below is from the Matlab Statistics Toolbox (see note above)
[Data_id, Centers] = kmeans(DataAll', model.nbStates);

% Setting means and covariance matrices
Mu = Centers';
Sigma = zeros(model.nbFrames*model.nbVar, model.nbFrames*model.nbVar, model.nbStates);
for i = 1 : model.nbStates
  idtmp = find(Data_id==i);
  model.Priors(i) = length(idtmp);
  Sigma(:,:,i) = cov(DataAll(:,idtmp)') + eye(size(DataAll,1))*diagRegularizationFactor;
end
model.Priors = model.Priors / sum(model.Priors);

%Reshape GMM parameters into a tensor 
for m = 1 : model.nbFrames
  for i = 1 : model.nbStates
    model.Mu(:,m,i) = Mu((m-1)*model.nbVar+1:m*model.nbVar,i); 
    model.Sigma(:,:,m,i) = Sigma((m-1)*model.nbVar+1:m*model.nbVar,(m-1)*model.nbVar+1:m*model.nbVar,i); 
  end
end


