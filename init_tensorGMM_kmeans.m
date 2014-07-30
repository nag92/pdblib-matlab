function model = init_tensorGMM_kmeans(Data, model)
% Initialization of the model with k-means. 
% Authors:	Sylvain Calinon, Tohid Alizadeh, 2013
%         http://programming-by-demonstration.org/

diagRegularizationFactor = 1E-4;

DataAll = reshape(Data, size(Data,1)*size(Data,2), size(Data,3)); %Matricization/flattening of tensor

%k-means clustering
[Data_id, Mu] = kmeansClustering(DataAll, model.nbStates);

for i=1:model.nbStates
  idtmp = find(Data_id==i);
  model.Priors(i) = length(idtmp);
  Sigma(:,:,i) = cov([DataAll(:,idtmp) DataAll(:,idtmp)]') + eye(size(DataAll,1))*diagRegularizationFactor;
end
model.Priors = model.Priors / sum(model.Priors);

%Reshape GMM parameters into a tensor
for m=1:model.nbFrames
  for i=1:model.nbStates
    model.Mu(:,m,i) = Mu((m-1)*model.nbVar+1:m*model.nbVar,i);
    model.Sigma(:,:,m,i) = Sigma((m-1)*model.nbVar+1:m*model.nbVar,(m-1)*model.nbVar+1:m*model.nbVar,i);
  end
end





