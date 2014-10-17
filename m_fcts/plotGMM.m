function h = plotGMM(Mu, Sigma, color)
% This function displays the parameters of a Gaussian Mixture Model (GMM).
% Inputs -----------------------------------------------------------------
%   o Mu:           D x K array representing the centers of K Gaussians.
%   o Sigma:        D x D x K array representing the covariance matrices of K Gaussians.
%   o color:        3 x 1 array representing the RGB color to use for the display.
%
% Author:	Sylvain Calinon, 2014
%         http://programming-by-demonstration.org/SylvainCalinon

nbStates = size(Mu,2);
nbDrawingSeg = 35;
lightcolor = min(color+0.3,1);
t = linspace(-pi, pi, nbDrawingSeg);

h=[];
for i=1:nbStates
	R = real(sqrtm(1.0.*Sigma(:,:,i)));
	X = R * [cos(t); sin(t)] + repmat(Mu(:,i), 1, nbDrawingSeg);
	h = [h patch(X(1,:), X(2,:), lightcolor, 'lineWidth', 1, 'EdgeColor', color)];
	h = [h plot(Mu(1,:), Mu(2,:), 'x', 'lineWidth', 2, 'markersize', 6, 'color', color)];
end
