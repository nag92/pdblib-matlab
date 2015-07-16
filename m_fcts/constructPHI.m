function [PHI,PHI1,PHI0,T,T1] = constructPHI(model,nbD,nbSamples)
%Construct PHI operator (big sparse matrix) used in trajectory-GMM, see Eq. (2.4.5) in doc/TechnicalReport.pdf
%Sylvain Calinon, 2015

T1 = nbD; %Number of datapoints in a demonstration
T = T1 * nbSamples; %Total number of datapoints
op1D = zeros(model.nbDeriv);
op1D(1,end) = 1;
for i=2:model.nbDeriv
	op1D(i,:) = (op1D(i-1,:) - circshift(op1D(i-1,:),[0,-1])) / model.dt;
end
op = zeros(T1*model.nbDeriv,T1);
op((model.nbDeriv-1)*model.nbDeriv+1:model.nbDeriv*model.nbDeriv,1:model.nbDeriv) = op1D;
PHI0 = zeros(T1*model.nbDeriv,T1);
for t=0:T1-model.nbDeriv
	PHI0 = PHI0 + circshift(op, [model.nbDeriv*t,t]);
end
%Handling of borders
for i=1:model.nbDeriv-1
	op(model.nbDeriv*model.nbDeriv+1-i,:)=0; op(:,i)=0;
	PHI0 = PHI0 + circshift(op, [-i*model.nbDeriv,-i]);
end
%Application to multiple dimensions and multiple demonstrations
PHI1 = kron(PHI0, eye(model.nbVarPos));
PHI = kron(eye(nbSamples), PHI1);