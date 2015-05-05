function r = reproduction_DS(DataIn, model, r, currPos)
% Reproduction with a virtual spring-damper system with constant impedance parameters
%
% Author: Sylvain Calinon, 2014
%         http://programming-by-demonstration.org/SylvainCalinon
%
% This source code is given for free! In exchange, I would be grateful if you cite
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
nbVarOut = length(currPos);

%% Reproduction with constant impedance parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x = currPos;
dx = zeros(nbVarOut,1);
for t=1:nbData
	L = [eye(nbVarOut)*model.kP, eye(nbVarOut)*model.kV];
	%Compute acceleration
	ddx =  -L * [x-r.currTar(:,t); dx]; %See Eq. (4.0.1) in doc/TechnicalReport.pdf
	%Update velocity and position
	dx = dx + ddx * model.dt;
	x = x + dx * model.dt;
	%Log data
	r.Data(:,t) = [DataIn(:,t); x];
	r.ddxNorm(t) = norm(ddx);
	r.kpDet(t) = det(L(:,1:nbVarOut));
	r.kvDet(t) = det(L(:,nbVarOut+1:end));
end



