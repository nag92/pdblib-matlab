function [Mu, Sigma] = gaussianProduct(model, p)

% Leonel Rozo, 2014
% 
% Compute the product of Gaussians for a task-parametrized model where the
% set of parameters are stored in the variable 'p'. 

% GMM products 
for i = 1 : model.nbStates
  % Reallocating  
  SigmaTmp = zeros(model.nbVar);
  MuTmp = zeros(model.nbVar,1);
  % Product of Gaussians
  for m = 1 : model.nbFrames 
    MuP = p(m).A * model.Mu(:,m,i) + p(m).b; 
    SigmaP = p(m).A * model.Sigma(:,:,m,i) * p(m).A'; 
    SigmaTmp = SigmaTmp + inv(SigmaP);
    MuTmp = MuTmp + SigmaP\MuP; 
  end
  Sigma(:,:,i) = inv(SigmaTmp);
  Mu(:,i) = Sigma(:,:,i) * MuTmp;    
end