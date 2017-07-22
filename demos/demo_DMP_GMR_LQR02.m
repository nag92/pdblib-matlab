function demo_DMP_GMR_LQR02
% Enhanced dynamic movement primitive (DMP) model trained with EM by using a Gaussian mixture 
% model (GMM) representation, with full covariance matrices coordinating the different variables 
% in the feature space, and by using the task-parameterized model formalism. After learning 
% (i.e., autonomous organization of the basis functions (position and spread), Gaussian mixture 
% regression (GMR) is used to regenerate the path of a spring-damper system, resulting in a 
% nonlinear force profile. The gains of the spring-damper system are further refined by LQR 
% based on the retrieved covariance information.  
% In this example, perturbations are added to show the benefit of encapsulating covariance 
% information to coordinate disturbance rejection. 
%
% Writing code takes time. Polishing it and making it available to others takes longer! 
% If some parts of the code were useful for your research of for a better understanding 
% of the algorithms, please reward the authors by citing the related publications, 
% and consider making your own research available in this way.
%
% @article{Calinon16JIST,
%   author="Calinon, S.",
%   title="A Tutorial on Task-Parameterized Movement Learning and Retrieval",
%   journal="Intelligent Service Robotics",
%		publisher="Springer Berlin Heidelberg",
%		doi="10.1007/s11370-015-0187-9",
%		year="2016",
%		volume="9",
%		number="1",
%		pages="1--29"
% }
% 
% Copyright (c) 2015 Idiap Research Institute, http://idiap.ch/
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
model.nbStates = 1; %Number of states in the GMM
model.nbVar = 3; %Number of variables [s,F1,F2] (decay term and perturbing force)
model.nbFrames = 1; %Number of candidate frames of reference (centered on goal position)
model.kP = 50; %Stiffness gain
model.kV = (2*model.kP)^.5; %Damping gain (with ideal underdamped damping ratio)
model.alpha = 1.0; %Decay factor
model.dt = 0.01; %Duration of time step
model.nbVarPos = model.nbVar-1; %Dimension of spatial variables
model.rFactor = 1E-5; %Weighting term for the minimization of control commands in LQR
nbData = 200; %Number of datapoints in a trajectory
nbSamples = 3; %Number of demonstrations

%Canonical system parameters
A = kron([0 1; 0 0], eye(model.nbVarPos)); %See Eq. (5.1.1) in doc/TechnicalReport.pdf
B = kron([0; 1], eye(model.nbVarPos)); %See Eq. (5.1.1) in doc/TechnicalReport.pdf
C = kron([1,0],eye(model.nbVarPos));
%Discretize system (Euler method)
Ad = A*model.dt + eye(size(A));
Bd = B*model.dt;
%Control cost matrix
R = eye(model.nbVar) * model.rFactor;
R = kron(eye(nbData),R);

%Create transformation matrix to compute xhat = x + dx*kV/kP + ddx/kP, see Eq. (4.0.2) in doc/TechnicalReport.pdf
K1d = [1, model.kV/model.kP, 1/model.kP];
K = kron(K1d,eye(model.nbVarPos));


%% Load handwriting data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
demos=[];
load('data/2Dletters/I.mat');
sIn(1) = 1; %Initialization of decay term
for t=2:nbData
	sIn(t) = sIn(t-1) - model.alpha * sIn(t-1) * model.dt; %Update of decay term (ds/dt=-alpha s)
end
DataDMP = zeros(model.nbVar,1,nbData*nbSamples);
Data=[];
for n=1:nbSamples
	%Task parameters (canonical coordinate system centered on the end-trajectory target)
	s(n).p(1).A = eye(model.nbVar);
	s(n).p(1).b = [0; demos{n}.pos(:,end)];
	%Demonstration data as [x;dx;ddx]
	s(n).Data = spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),nbData)); %Resampling
	Data = [Data s(n).Data]; %Original data
	s(n).Data = [s(n).Data; gradient(s(n).Data)/model.dt]; %Velocity computation
	s(n).Data = [s(n).Data; gradient(s(n).Data(end-model.nbVarPos+1:end,:))/model.dt]; %Acceleration computation
	s(n).Data = [sIn; K*s(n).Data]; %xhat computation
	s(n).Data = s(n).p(1).A \ (s(n).Data - repmat(s(n).p(1).b,1,nbData)); %Observation from the perspective of the frame
	DataDMP(:,1,(n-1)*nbData+1:n*nbData) = s(n).Data; %Training data as [s;xhat]
end


%% Learning and reproduction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%model = init_tensorGMM_kmeans(DataDMP, model); 
model = init_tensorGMM_timeBased(DataDMP, model); 

%Learning of full covariances
%model = EM_tensorGMM(DataDMP, model);

%Learning of covariances structures as factor analyzers
model.nbFA = 1;
model = EM_tensorMFA(DataDMP, model);


%Task-adaptive spring-damper attractor path retrieval
r(n).p(1).A = eye(model.nbVar);
r(n).p(1).b = s(n).p(1).b;
[r(n).Mu, r(n).Sigma] = productTPGMM0(model, r(n).p); %See Eq. (6.0.5), (6.0.6) and (6.0.7) in doc/TechnicalReport.pdf
r(n).Priors = model.Priors;
r(n).nbStates = model.nbStates;
[r(1).currTar, r(1).currSigma] = GMR(r(n), sIn, 1, 2:model.nbVar); %See Eq. (3.0.2) to (3.0.5) in doc/TechnicalReport.pdf

%LQR tracking (discrete version)
Q = zeros(model.nbVarPos*2, model.nbVarPos*2);
R = eye(model.nbVarPos) * model.rFactor;
P = zeros(model.nbVarPos*2, model.nbVarPos*2, nbData);
P(1:model.nbVarPos,1:model.nbVarPos,end) = inv(r(1).currSigma(:,:,nbData));
for t=nbData-1:-1:1
	Q(1:model.nbVarPos,1:model.nbVarPos) = inv(r(1).currSigma(:,:,t));
	P(:,:,t) = Q - Ad' * (P(:,:,t+1) * Bd / (Bd' * P(:,:,t+1) * Bd + R) * Bd' * P(:,:,t+1) - P(:,:,t+1)) * Ad;
end

%External perturbation force
Fpert = zeros(model.nbVarPos,nbData);
%Fpert(:,1:nbData/4) = repmat([0; 1E4], 1, nbData/4); %Perturbation force during the first 1/4 part of the motion
%Fpert(:,5) = [4E4; 0]; %Impulse perturbation force at the beginning of the motion 

%Motion retrieval 
x = Data(1:model.nbVarPos,1) + [-5; -2]; %Offset for the starting point
dx = zeros(model.nbVarPos,1);
for t=1:nbData
	r(1).Data(:,t) = x;
	K = (Bd' * P(:,:,t) * Bd + R) \ Bd' * P(:,:,t) * Ad;
	ddx = K * ([r(1).currTar(:,t); zeros(model.nbVarPos,1)] - [x; dx]) + Fpert(:,t);
	dx = dx + ddx * model.dt;
	x = x + dx * model.dt;
	r(1).detKp(t) = det(K(:,1:model.nbVarPos));
end	

%Motion retrieval 
x = Data(1:model.nbVarPos,1) + [-5; -2]; %Offset for the starting point
dx = zeros(model.nbVarPos,1);
for t=1:nbData
	r(2).Data(:,t) = x;
	K = (Bd' * P(:,:,t) * Bd + R) \ Bd' * P(:,:,t) * Ad;
	%Corresponding scalar stiffness and damping gains (for comparison purpose)
	K(:,1:model.nbVarPos) = eye(model.nbVarPos) * det(K(:,1:model.nbVarPos))^(1/model.nbVarPos);
	K(:,model.nbVarPos+1:end) = eye(model.nbVarPos) * det(K(:,model.nbVarPos+1:end))^(1/model.nbVarPos);
	
	ddx = K * ([r(1).currTar(:,t); zeros(model.nbVarPos,1)] - [x; dx]) + Fpert(:,t);
	dx = dx + ddx * model.dt;
	x = x + dx * model.dt;
end	


%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10,10,1300,450],'color',[1 1 1]); 
xx = round(linspace(1,64,model.nbStates));
clrmap = colormap('jet')*0.5;
clrmap = min(clrmap(xx,:),.9);

%Activation of the basis functions
for i=1:model.nbStates
	h(i,:) = model.Priors(i) * gaussPDF(sIn, model.Mu(1,i), model.Sigma(1,1,i));
end
h = h ./ repmat(sum(h,1)+realmin, model.nbStates, 1);

%Spatial plot
subplot(2,4,[1,5]); hold on; axis off;
plot(Data(1,:), Data(2,:), '.','markersize',8,'color',[.7 .7 .7]);
plot(r(1).currTar(1,:), r(1).currTar(2,:), '-','linewidth',2,'color',[1 .7 .7]); %Attractor path
plot(r(1).Data(1,:), r(1).Data(2,:), '-','linewidth',3,'color',[0 .8 0]); %Retrieved path
plot(r(2).Data(1,:), r(2).Data(2,:), '-','linewidth',2,'color',[.8 0 0]); %Retrieved path
plot(r(n).p(1).b(2), r(n).p(1).b(3), 'k+','linewidth',2,'markersize',12);
axis equal; 

%Timeline plot of the nonlinear perturbing force
subplot(2,4,[2:4]); hold on;
for n=1:nbSamples
	plot(sIn, DataDMP(3,(n-1)*nbData+1:n*nbData), '-','linewidth',2,'color',[.7 .7 .7]);
end
for i=1:model.nbStates
	plotGMM(model.Mu([1,3],i), model.Sigma([1,3],[1,3],i), clrmap(i,:), .7);
end
plot(sIn, r(1).currTar(2,:), '-','linewidth',2,'color',[.8 0 0]);
%axis([0 1 min(DataDMP(3,:)) max(DataDMP(3,:))]);
ylabel('$\hat{x}_2$','fontsize',16,'interpreter','latex');
view(180,-90);

%Timeline plot of the evolution of stiffness
subplot(2,4,[6:8]); hold on;
plot(sIn, r(1).detKp, '-','linewidth',2,'color',[0 0 0]);
axis([0 1 min(r(1).detKp) max(r(1).detKp)]);
xlabel('$s$','fontsize',16,'interpreter','latex'); 
ylabel('$|Kp|$','fontsize',16,'interpreter','latex');
view(180,-90);

%print('-dpng','graphs/demo_DMP_GMR_LQR02.png');
pause;
close all;
