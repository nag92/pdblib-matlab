function r = estimateAttractorPath(DataIn, model, r)
% Estimation of an attractor path from a task-parameterized GMM and a set of candidate frames.
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

nbData = size(DataIn,2);
in = 1:size(DataIn,1);
out = in(end)+1:model.nbVar;
nbVarOut = length(out);

%% GMR to estimate attractor path and associated variations
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%GMM products 
for i=1:model.nbStates 
  SigmaTmp = zeros(model.nbVar);
  MuTmp = zeros(model.nbVar,1);
  for m=1:model.nbFrames 
    MuP = r.p(m).A * model.Mu(:,m,i) + r.p(m).b; 
    SigmaP = r.p(m).A * model.Sigma(:,:,m,i) * r.p(m).A'; 
    SigmaTmp = SigmaTmp + inv(SigmaP);
    MuTmp = MuTmp + SigmaP\MuP; 
  end
  r.Sigma(:,:,i) = inv(SigmaTmp);
  r.Mu(:,i) = r.Sigma(:,:,i) * MuTmp;
end

%GMR
MuTmp = zeros(nbVarOut,model.nbStates);
for t=1:nbData
  %Compute activation weight
  for i=1:model.nbStates
    r.H(i,t) = model.Priors(i) * gaussPDF(DataIn(:,t), r.Mu(in,i), r.Sigma(in,in,i)); 
  end
  r.H(:,t) = r.H(:,t)/sum(r.H(:,t));
  %Evaluate the current target 
  currTar = zeros(nbVarOut,1);
  currSigma = zeros(nbVarOut,nbVarOut);   
  %Compute expected conditional means
	for i=1:model.nbStates
		MuTmp(:,i) = r.Mu(out,i) + r.Sigma(out,in,i)/r.Sigma(in,in,i) * (DataIn(:,t)-r.Mu(in,i));
		currTar = currTar + r.H(i,t) * MuTmp(:,i);
	end
	%Compute expected conditional covariances
	for i=1:model.nbStates
		SigmaTmp = r.Sigma(out,out,i) - r.Sigma(out,in,i)/r.Sigma(in,in,i) * r.Sigma(in,out,i);
		currSigma = currSigma + r.H(i,t) * (SigmaTmp + MuTmp(:,i)*MuTmp(:,i)');  
	 	for j=1:model.nbStates
		 	currSigma = currSigma - r.H(i,t)*r.H(j,t) * (MuTmp(:,i)*MuTmp(:,j)');  
	 	end
	end
  r.currTar(:,t) = currTar; 
  r.currSigma(:,:,t) = currSigma;
end

