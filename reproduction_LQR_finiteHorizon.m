function r = reproduction_LQR_finiteHorizon(DataIn, model, r, currPos, rFactor, Sfinal)
% Reproduction with a linear quadratic regulator of finite horizon
%
% Authors: Sylvain Calinon and Danilo Bruno, 2014
%          http://programming-by-demonstration.org/SylvainCalinon
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

%% LQR with cost = sum_t X(t)' Q(t) X(t) + u(t)' R u(t) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Definition of a double integrator system (DX = A X + B u with X = [x; dx])
A = kron([0 1; 0 0], eye(nbVarOut));
B = kron([0; 1], eye(nbVarOut));
%Initialize Q and R weighting matrices
Q = zeros(nbVarOut*2,nbVarOut*2,nbData);
for t=1:nbData
  Q(1:nbVarOut,1:nbVarOut,t) = inv(r.currSigma(:,:,t)); 
end
R = eye(nbVarOut) * rFactor;

if nargin<6
  %Sfinal = B*R*[eye(nbVarOut)*model.kP, eye(nbVarOut)*model.kV]
  Sfinal = B*R*[eye(nbVarOut)*0, eye(nbVarOut)*0]; %final feedback terms (boundary conditions)
end

%Auxiliary variables to minimize the cost function
S = zeros(nbVarOut*2, nbVarOut*2, nbData);
d = zeros(nbVarOut*2, nbData); %For optional feedforward term computation

%Feedback term
L = zeros(nbVarOut, nbVarOut*2, nbData);
%Compute S_T from the desired final feedback gains L_T,  
S(:,:,nbData) = Sfinal; 

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
  S(:,:,t) = S(:,:,t+1) + model.dt * (A'*S(:,:,t+1) + S(:,:,t+1)*A - S(:,:,t+1) * B * (R\B') * S(:,:,t+1) + Q(:,:,t+1));  
  %Optional feedforward term computation
	d(:,t) = d(:,t+1) + model.dt * ((A'-S(:,:,t+1)*B*(R\B'))*d(:,t+1) +  S(:,:,t+1)*dtar(:,t+1) - S(:,:,t+1)*A*tar(:,t+1)); 
end
%Computation of the feedback term L in u=-LX+M
for t=1:nbData
  L(:,:,t) = R\B' * S(:,:,t); 
  M(:,t) = R\B' * d(:,t); %Optional feedforward term computation
end

%% Reproduction with varying impedance parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x = currPos;
dx = zeros(nbVarOut,1);
for t=1:nbData
  %Compute acceleration (with only feedback term)
  %ddx =  -L(:,:,t) * [x-r.currTar(:,t); dx];
  
  %Compute acceleration (with both feedback and feedforward terms)
  ddx =  -L(:,:,t) * [x-r.currTar(:,t); dx] + M(:,t);
  r.FB(:,t) = -L(:,:,t) * [x-r.currTar(:,t); dx];
  r.FF(:,t) = M(:,t);
  
  %ddx =  -L * ([x;dx]-tar(:,t)) + M;
  %r.FB(:,t) = -L * ([x;dx]-tar(:,t));
  %r.FF(:,t) = M;
  
  %Update velocity and position
  dx = dx + ddx * model.dt;
  x = x + dx * model.dt;
  %Log data
  r.Data(:,t) = [DataIn(:,t); x]; 
  r.ddxNorm(t) = norm(ddx);
  r.kpDet(t) = det(L(:,1:nbVarOut,t));
  r.kvDet(t) = det(L(:,nbVarOut+1:end,t));
end



