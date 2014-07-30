function [idList, Mu] = kmeansClustering(Data, nbStates)
% Initialization of the model with k-means. 
% Authors:	Sylvain Calinon, Tohid Alizadeh, 2013
%         http://programming-by-demonstration.org/

%Criterion to stop the EM iterative update
cumdist_threshold = 1e-10;
maxIter = 100;

%Initialization of the parameters
[nbVar, nbData] = size(Data);
cumdist_old = -realmax;
nbStep = 0;

idTmp = randperm(nbData);
Mu = Data(:,idTmp(1:nbStates));

%k-means iterations
while 1
  %E-step %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  for i=1:nbStates
    %Compute distances
    distTmp(:,i) = sum((Data-repmat(Mu(:,i),1,nbData)).^2);
  end
  [vTmp,idList] = min(distTmp,[],2);
  cumdist = sum(vTmp);
  %M-step %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  for i=1:nbStates
    %Update the centers
    Mu(:,i) = mean(Data(:,idList==i),2);
  end
  %Stopping criterion %%%%%%%%%%%%%%%%%%%%
  if abs(cumdist-cumdist_old) < cumdist_threshold
    break;
  end
  cumdist_old = cumdist;
  nbStep = nbStep+1;
%   if nbStep>maxIter
%     disp(['Maximum number of iterations, ' num2str(maxIter) 'is reached']);
%     break;
%   end
end

%disp(['Kmeans stopped after ' num2str(nbStep) ' steps.']); 


