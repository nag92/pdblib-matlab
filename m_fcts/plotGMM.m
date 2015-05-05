function h = plotGMM(Mu, Sigma, color, valAlpha)
% This function displays the parameters of a Gaussian Mixture Model (GMM).
% Inputs -----------------------------------------------------------------
%   o Mu:           D x K array representing the centers of K Gaussians.
%   o Sigma:        D x D x K array representing the covariance matrices of K Gaussians.
%   o color:        3 x 1 array representing the RGB color to use for the display.
%   o valAlpha:     transparency factor (optional).
%
% Author:	Sylvain Calinon, 2014
%         http://programming-by-demonstration.org/SylvainCalinon

nbStates = size(Mu,2);
nbDrawingSeg = 35;
lightcolor = min(color+0.5,1);
t = linspace(-pi, pi, nbDrawingSeg);

h=[];
for i=1:nbStates
	%R = real(sqrtm(1.0.*Sigma(:,:,i)));
	[V,D] = eig(Sigma(:,:,i));
	R = real(V*D.^.5);
	X = R * [cos(t); sin(t)] + repmat(Mu(:,i), 1, nbDrawingSeg);
	if nargin>3 %Plot with alpha transparency
		h = [h patch(X(1,:), X(2,:), lightcolor, 'lineWidth', 1, 'EdgeColor', color, 'facealpha', valAlpha,'edgealpha', valAlpha)];
		MuTmp = [cos(t); sin(t)] * 0.006 + repmat(Mu(:,i),1,nbDrawingSeg);
		h = [h patch(MuTmp(1,:), MuTmp(2,:), color, 'LineStyle', 'none', 'facealpha', valAlpha)];  
	else %Plot without transparency
		h = [h patch(X(1,:), X(2,:), lightcolor, 'lineWidth', 1, 'EdgeColor', color)];
		h = [h plot(Mu(1,:), Mu(2,:), '.', 'lineWidth', 2, 'markersize', 6, 'color', color)];
	end
end
