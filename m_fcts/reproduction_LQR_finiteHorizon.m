function r = reproduction_LQR_finiteHorizon(DataIn, model, r, currPos, rFactor, Pfinal)
% Reproduction with a linear quadratic regulator of finite horizon
%
% Authors: Sylvain Calinon and Danilo Bruno, 2014
%          http://programming-by-demonstration.org
%
% This source code is given for free! In exchange, we would be grateful if you cite
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
nbVarOut = model.nbVar - size(DataIn,1);

%% LQR with cost = sum_t X(t)' Q(t) X(t) + u(t)' R u(t) (See Eq. (5.0.2) in doc/TechnicalReport.pdf)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Definition of a double integrator system (DX = A X + B u with X = [x; dx])
A = kron([0 1; 0 0], eye(nbVarOut)); %See Eq. (5.1.1) in doc/TechnicalReport.pdf
B = kron([0; 1], eye(nbVarOut)); %See Eq. (5.1.1) in doc/TechnicalReport.pdf
%Initialize Q and R weighting matrices
Q = zeros(nbVarOut*2,nbVarOut*2,nbData);
for t=1:nbData
	Q(1:nbVarOut,1:nbVarOut,t) = inv(r.currSigma(:,:,t));
end
R = eye(nbVarOut) * rFactor;

if nargin<6
	%Pfinal = B*R*[eye(nbVarOut)*model.kP, eye(nbVarOut)*model.kV]
	Pfinal = B*R*[eye(nbVarOut)*0, eye(nbVarOut)*0]; %final feedback terms (boundary conditions)
end

%Auxiliary variables to minimize the cost function
P = zeros(nbVarOut*2, nbVarOut*2, nbData);
d = zeros(nbVarOut*2, nbData); %For optional feedforward term computation

%Feedback term
L = zeros(nbVarOut, nbVarOut*2, nbData);
%Compute P_T from the desired final feedback gains L_T,
P(:,:,nbData) = Pfinal;

%Compute derivative of target path
%dTar = diff(r.currTar, 1, 2); %For optional feedforward term computation

%Variables for feedforward term computation (optional for movements with low dynamics)
tar = [r.currTar; zeros(nbVarOut,nbData)];
dtar = gradient(tar,1,2)/model.dt;
%tar = [r.currTar; gradient(r.currTar,1,2)/model.dt];
%dtar = gradient(tar,1,2)/model.dt;
%tar = [r.currTar; diff([r.currTar(:,1) r.currTar],1,2)/model.dt];
%dtar = diff([tar tar(:,1)],1,2)/model.dt;


%Backward integration of the Riccati equation and additional equation
for t=nbData-1:-1:1
	P(:,:,t) = P(:,:,t+1) + model.dt * (A'*P(:,:,t+1) + P(:,:,t+1)*A - P(:,:,t+1)*B*(R\B')*P(:,:,t+1) + Q(:,:,t+1)); %See Eq. (5.1.11) in doc/TechnicalReport.pdf
	%Optional feedforward term computation
	d(:,t) = d(:,t+1) + model.dt * ((A'-P(:,:,t+1)*B*(R\B'))*d(:,t+1) + P(:,:,t+1)*dtar(:,t+1) - P(:,:,t+1)*A*tar(:,t+1)); %See Eq. (5.1.29) in doc/TechnicalReport.pdf
end
%Computation of the feedback term L and feedforward term M in u=-LX+M
for t=1:nbData
	L(:,:,t) = R\B' * P(:,:,t); %See Eq. (5.1.30) in doc/TechnicalReport.pdf
	M(:,t) = R\B' * d(:,t); %Optional feedforward term computation (See Eq. (5.1.30) in doc/TechnicalReport.pdf)
end

%% Reproduction with varying impedance parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x = currPos;
dx = zeros(nbVarOut,1);
for t=1:nbData
	%Compute acceleration (with only feedback term)
	%ddx =  -L(:,:,t) * [x-r.currTar(:,t); dx];
	
	%Compute acceleration (with both feedback and feedforward terms)
	ddx =  -L(:,:,t) * [x-r.currTar(:,t); dx] + M(:,t); %See Eq. (5.1.30) in doc/TechnicalReport.pdf
	r.FB(:,t) = -L(:,:,t) * [x-r.currTar(:,t); dx];
	r.FF(:,t) = M(:,t);
	
	%Update velocity and position
	dx = dx + ddx * model.dt;
	x = x + dx * model.dt;
	%Log data
	r.Data(:,t) = [DataIn(:,t); x];
	r.ddxNorm(t) = norm(ddx);
	r.kpDet(t) = det(L(:,1:nbVarOut,t));
	r.kvDet(t) = det(L(:,nbVarOut+1:end,t));	
end


