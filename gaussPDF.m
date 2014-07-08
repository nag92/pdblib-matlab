function prob = gaussPDF(Data, Mu, Sigma)
% Likelihood of datapoint(s) to be generated by a Gaussian parameterized by center and covariance.
%
% Inputs -----------------------------------------------------------------
%   o Data:  D x N array representing N datapoints of D dimensions.
%   o Mu:    D x 1 vector representing the center of the Gaussian.
%   o Sigma: D x D array representing the covariance matrix of the Gaussian.
% Output -----------------------------------------------------------------
%   o prob:  1 x N vector representing the likelihood of the N datapoints.     
%
% Author:	Sylvain Calinon, 2014
%         http://programming-by-demonstration.org/SylvainCalinon

[nbVar,nbData] = size(Data);

Data = Data' - repmat(Mu',nbData,1);
prob = sum((Data/Sigma).*Data, 2);
prob = exp(-0.5*prob) / sqrt((2*pi)^nbVar * (abs(det(Sigma))+realmin));
