function demo_ergodicControl_3D01
% 3D ergodic control with a spatial distribution described as a GMM, inspired by G. Mathew and I. Mezic, 
% "Spectral Multiscale Coverage: A Uniform Coverage Algorithm for Mobile Sensor Networks", CDC'2009 
%
% If this code is useful for your research, please cite the related publication:
% @incollection{Calinon19MM,
%   author="Calinon, S.",
%   title="Mixture Models for the Analysis, Edition, and Synthesis of Continuous Time Series",
%   booktitle="Mixture Models and Applications",
%   publisher="Springer",
%   editor="Bouguila, N. and Fan, W.", 
%   year="2019"
% }
%
% Copyright (c) 2019 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon, http://calinon.ch/
% 
% This file is part of PbDlib, http://www.idiap.ch/software/pbdlib/
% 
% PbDlib is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License version 3 as
% published by the Free Software Foundation.
% 
% PbDlib is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with PbDlib. If not, see <http://www.gnu.org/licenses/>.

addpath('./m_fcts/');


%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nbData = 4000; %Number of datapoints
nbFct = 10; %Number of basis functions along x and y
nbVar = 3; %Dimension of datapoint
nbStates = 2; %Number of Gaussians to represent the spatial distribution
sp = (nbVar + 1) / 2; %Sobolev norm parameter
dt = 1E-2; %Time step
xlim = [0; 1]; %Domain limit for each dimension (considered to be 1 for each dimension in this implementation)
L = (xlim(2) - xlim(1)) * 2; %Size of [-xlim(2),xlim(2)]
om = 2 .* pi ./ L; %omega
u_max = 1E1; %Maximum speed allowed 
x = [.7; .1; .5]; %Initial position

%Desired spatial distribution represented as a mixture of Gaussians
Mu(:,1) = [.4; .5; .7]; 
Sigma(:,:,1) = eye(nbVar).*1E-2; 
Mu(:,2) =  [.7; .4; .3]; 
Sigma(:,:,2) = [.1;.2;.3]*[.1;.2;.3]' .*8E-1 + eye(nbVar)*1E-3; 
Priors = ones(1,nbStates) ./ nbStates; %Mixing coefficients


%% Compute Fourier series coefficients phi_k of desired spatial distribution
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[xx, yy, zz] = ndgrid(1:nbFct, 1:nbFct, 1:nbFct);
rg = 0:nbFct-1;
[KX(1,:,:,:), KX(2,:,:,:), KX(3,:,:,:)] = ndgrid(rg, rg, rg);
Lambda = (KX(1,:).^2 + KX(2,:).^2 + KX(3,:).^2 + 1)'.^-sp; %Weighting vector (Eq.(15))

HK = L^nbVar; %Rescaling term (as scalar)
% hk = [1; sqrt(.5)*ones(nbFct-1,1)];
% HK = hk(xx,1) .* hk(yy,1) .* hk(zz,1); %Rescaling term (as normalizing matrix)

%Explicit description of phi_k by exploiting the Fourier transform properties of Gaussians (optimized version by exploiting symmetries)
%Enumerate symmetry operations for 2D signal ([-1,-1],[-1,1],[1,-1] and [1,1]), and removing redundant ones -> keeping ([-1,-1],[-1,1])
op = hadamard(2^(nbVar-1));
op = op(1:nbVar,:);
%Compute phi_k
w = KX(:,:) .* om;
phi = zeros(nbFct^nbVar, 1);
for k=1:nbStates
	for n=1:size(op,2)
		MuTmp = diag(op(:,n)) * Mu(:,k); %Eq.(20)
		SigmaTmp = diag(op(:,n)) * Sigma(:,:,k) * diag(op(:,n))'; %Eq.(20)
		phi = phi + Priors(k) .* cos(w'*MuTmp) .* exp(diag(-.5 * w' * SigmaTmp * w)); %Eq.(21)
	end
end
phi = phi ./ HK ./ size(op,2);


%% Ergodic control 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ckt = zeros(nbFct^nbVar, 1);
for t=1:nbData
	r.x(:,t) = x; %Log data
	
	%Fourier basis functions and derivatives for each dimension (only cosine part on [0,L/2] is computed since the signal is even and real by construction) 
	fx = cos(x * rg .* om); %Eq.(18)
	dfx = -sin(x * rg .* om) .* repmat(rg,nbVar,1) .* om;
	
% 	%Fourier basis functions and derivatives for each dimension (only real part on [0,L/2] is computed since the signal is even and real by construction) 
% 	fx = real(exp(-i .* x * rg .* om)); %Eq.(18)
% 	dfx = real(-exp(-i .* x * rg .* om) .* i .* repmat(rg,nbVar,1) .* om);

	gradf = [dfx(1,xx).*fx(2,yy).*fx(3,zz); fx(1,xx).*dfx(2,yy).*fx(3,zz); fx(1,xx).*fx(2,yy).*dfx(3,zz)]; %gradient of basis functions
	ckt = ckt + (fx(1,xx) .* fx(2,yy) .* fx(3,zz))' ./ HK;	%ckt./t are the Fourier series coefficients along trajectory (Eq.(17))
	
% 	%Controller with ridge regression formulation
% 	u = -gradf * (Lambda .* (ckt./t - phi)) .* t .* 1E-1; %Velocity command
	
	%Controller with constrained velocity norm
	u = -gradf * (Lambda .* (ckt./t - phi)); %Eq.(24)
	u = u .* u_max ./ (norm(u)+1E-1); %Velocity command
	
	x = x + u .* dt; %Update of position
end


%% Plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10,10,1200,1200]); hold on; axis off; rotate3d on;
plotGMM3D(Mu, Sigma, [.2 .2 .2], .3, 2);
plot3(Mu(1,:), Mu(2,:), Mu(3,:), '.','markersize',15,'color',[0 0 0]);
plot3(r.x(1,:), r.x(2,:), r.x(3,:), '-','linewidth',1,'color',[0 0 0]);
plot3(r.x(1,1), r.x(2,1), r.x(3,1), '.','markersize',15,'color',[0 0 0]);
axis([xlim(1),xlim(2),xlim(1),xlim(2),xlim(1),xlim(2)]); axis equal; axis vis3d; view(60,25);
% plot3Dframe(eye(3).*.3, ones(3,1).*.2);
% print('-dpng','graphs/ergodicControl_3D01.png'); 

pause;
close all;

