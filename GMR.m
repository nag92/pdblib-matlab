function [expData, expSigma, H] = GMR(model, DataIn, in, out)
%Gaussian mixture regression (GMR)
%Sylvain Calinon, Danilo Bruno, 2014

nbData = size(DataIn,2);
nbVarOut = length(out);

MuTmp = zeros(nbVarOut,model.nbStates);
expData = zeros(nbVarOut,nbData);
expSigma = zeros(nbVarOut,nbVarOut,nbData);
for t=1:nbData
  %Compute activation weight
  %See Eq. (3.0.5) in doc/TechnicalReport.pdf
  for i=1:model.nbStates
    H(i,t) = model.Priors(i) * gaussPDF(DataIn(:,t), model.Mu(in,i), model.Sigma(in,in,i)); 
  end
  H(:,t) = H(:,t)/sum(H(:,t));
  %Compute expected conditional means
  %See Eq. (3.0.3) in doc/TechnicalReport.pdf
	for i=1:model.nbStates
		MuTmp(:,i) = model.Mu(out,i) + model.Sigma(out,in,i)/model.Sigma(in,in,i) * (DataIn(:,t)-model.Mu(in,i));
		expData(:,t) = expData(:,t) + H(i,t) * MuTmp(:,i);
	end
	%Compute expected conditional covariances
	%See Eq. (3.0.4) in doc/TechnicalReport.pdf
	for i=1:model.nbStates
		SigmaTmp = model.Sigma(out,out,i) - model.Sigma(out,in,i)/model.Sigma(in,in,i) * model.Sigma(in,out,i);
		expSigma(:,:,t) = expSigma(:,:,t) + H(i,t) * (SigmaTmp + MuTmp(:,i)*MuTmp(:,i)');  
	 	for j=1:model.nbStates
		 	expSigma(:,:,t) = expSigma(:,:,t) - H(i,t)*H(j,t) * (MuTmp(:,i)*MuTmp(:,j)');  
	 	end
	end
end

