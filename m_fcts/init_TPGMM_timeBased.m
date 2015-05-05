function model = init_TPGMM_timeBased(Data, model)
% Author:	Sylvain Calinon, 2014
%         http://programming-by-demonstration.org/SylvainCalinon

diagRegularizationFactor = 1E-4;
nbVar = size(Data,1); %nbVar is used instead of model.nbVar to be compatible with TPGMM with frames of different sizes

DataAll = reshape(Data, size(Data,1)*size(Data,2), size(Data,3)); %Matricization/flattening of tensor

TimingSep = linspace(min(DataAll(1,:)), max(DataAll(1,:)), model.nbStates+1);
Mu = zeros(model.nbFrames*nbVar, model.nbStates);
Sigma = zeros(model.nbFrames*nbVar, model.nbFrames*nbVar, model.nbStates);
for i=1:model.nbStates
	idtmp = find( DataAll(1,:)>=TimingSep(i) & DataAll(1,:)<TimingSep(i+1));
	Mu(:,i) = mean(DataAll(:,idtmp),2);
	Sigma(:,:,i) = cov(DataAll(:,idtmp)') + eye(size(DataAll,1))*diagRegularizationFactor;
	model.Priors(i) = length(idtmp);
end
model.Priors = model.Priors / sum(model.Priors);

%Reshape GMM parameters into a tensor
for m=1:model.nbFrames
	for i=1:model.nbStates
		model.Mu(:,m,i) = Mu((m-1)*nbVar+1:m*nbVar,i);
		model.Sigma(:,:,m,i) = Sigma((m-1)*nbVar+1:m*nbVar,(m-1)*nbVar+1:m*nbVar,i);
	end
end

