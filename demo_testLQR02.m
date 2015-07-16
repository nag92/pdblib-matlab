function demo_testLQR02
% Test of the linear quadratic regulation (evaluation of the damping ratio found by the system)
%
% Sylvain Calinon, 2015
% http://programming-by-demonstration.org/lib/
%
% This source code is given for free! In exchange, I would be grateful if you cite
% the following reference in any academic publication that uses this code or part of it:
%
% @article{Calinon15,
%   author="Calinon, S.",
%   title="A tutorial on task-parameterized movement learning and retrieval",
%   year="2015",
% }

addpath('./m_fcts/');


%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model.nbVar = 2; %Dimension of the datapoints in the dataset (here: t,x1)
model.dt = 0.01; %Time step
nbData = 1000; %Number of datapoints
nbRepros = 1; %Number of reproductions with new situations randomly generated
rFactor = 1E-1; %Weighting term for the minimization of control commands in LQR


%% Reproduction with LQR
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Reproductions with LQR...');
DataIn = [1:nbData] * model.dt;
a.currTar = ones(1,nbData);
a.currSigma = ones(1,1,nbData); %-> LQR with cost X'X + u'u
for n=1:nbRepros
	%r(n) = reproduction_LQR_finiteHorizon(model, a, 0, rFactor);
	r(n) = reproduction_LQR_infiniteHorizon(model, a, 0, rFactor);
	r(n).Data = [DataIn; r(n).Data];
end


%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[20,50,1300,500]);
hold on; box on;
%Plot target
plot(r(1).Data(1,:), a.currTar, 'r-', 'linewidth', 2);
for n=1:nbRepros
	%Plot trajectories
	plot(r(n).Data(1,:), r(n).Data(2,:), 'k-', 'linewidth', 2);
end
xlabel('t'); ylabel('x_1');

figure;
%Plot norm of control commands
subplot(1,3,1); hold on;
for n=1:nbRepros
	plot(DataIn, r(n).ddxNorm, 'k-', 'linewidth', 2);
end
xlabel('t'); ylabel('|ddx|');
%Plot stiffness
subplot(1,3,2); hold on;
for n=1:nbRepros
	plot(DataIn, r(n).kpDet, 'k-', 'linewidth', 2);
end
xlabel('t'); ylabel('kp');
%Plot stiffness/damping ratio (equals to optimal control ratio 1/2^.5)
subplot(1,3,3); hold on;
for n=1:nbRepros
	%Ideal damping ratio of 1/2^.5 = 0.7071, corresponding to r(n).kvDet(1) = (2*r(n).kpDet(1))^.5
	dampingRatio = r(n).kvDet(:) ./ (2*r(n).kpDet(:).^.5);
	plot(DataIn, dampingRatio, 'k-', 'linewidth', 2);
end
xlabel('t'); ylabel('Damping ratio');

%print('-dpng','graphs/demo_testLQR02.png');
%pause;
%close all;

