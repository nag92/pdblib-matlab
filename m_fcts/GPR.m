function [yd, SigmaOut] = GPR(q, y, qd, p, covopt)
%Gaussian process regression (GPR), see Eq. (7.1.2) in doc/TechnicalReport.pdf
%Sylvain Calinon, 2015

%Kernel parameters
if nargin<4
	%p(1)=1; p(2)=1; p(3)=1E-5;
	p(1)=1; p(2)=1E-1; p(3)=1E-3;
end

%Covariance computation
if nargin<5
	covopt = 1;
end
	
diagRegularizationFactor = 1E-4; %Regularization term is optional, see Eq. (2.1.2) in doc/TechnicalReport.pdf

% %Linear least-squares regression
% vOut = y * (pinv(q)*vIn);

% %Recenter data
% qmean = mean(q,2);
% q = q - repmat(qmean,1,size(q,2));
% qd = qd - repmat(qmean,1,size(vIn,2)); 
% ymean = mean(y,2);
% y = y - repmat(ymean,1,size(q,2));

%GPR with exp() kernel
M = pdist2(q', q');
Md = pdist2(qd', q');
K = p(1) * exp(-p(2) * M.^2);
Kd = p(1) * exp(-p(2) * Md.^2);
invK = pinv(K + p(3) * eye(size(K))); 

%Output
yd = (Kd * invK * y')'; % + repmat(ymean,1,size(qd,2)); 

if nargout>1
	SigmaOut = zeros(size(yd,1), size(yd,1), size(yd,2));
	if covopt==0
		%Evaluate Sigma as in Rasmussen, 2006
		Mdd = pdist2(qd',qd');
		Kdd = exp(-Mdd.^2);
		S = Kdd - Kd * invK * Kd';
		for t=1:size(yd,2)
			SigmaOut(:,:,t) = eye(size(yd,1)) * S(t,t); 
		end
	else
		%Evaluate Sigma as in GMR
		%nbSamples = size(y,2) / size(yd,2);
		%yd = repmat(yd,1,nbSamples);
		for t=1:size(yd,2)
			W = diag(K(t,:) * invK);
			ym = repmat(yd(:,t), 1, size(y,2));
			%SigmaOut(:,:,t) = (y-yd) * W * (y-yd)' + eye(size(vOut,1))*diagRegularizationFactor;  
			SigmaOut(:,:,t) = (y-ym) * W * (y-ym)' + eye(size(yd,1))*diagRegularizationFactor; 
		end
	end
end


