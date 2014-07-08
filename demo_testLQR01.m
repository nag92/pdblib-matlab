function demo_testLQR01
% Test of the linear quadratic regulation
%
% Author:	Sylvain Calinon, 2014
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

%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model.nbVar = 2; %Dimension of the datapoints in the dataset (here: t,x1)
model.dt = 0.01; %Time step 
nbData = 500; %Number of datapoints
nbRepros = 3; %Number of reproductions with new situations randomly generated
rFactor = 1E-2; %Weighting term for the minimization of control commands in LQR

%% Reproduction with LQR 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Reproductions with LQR...');
DataIn = [1:nbData] * model.dt;
a.currTar = ones(1,nbData); 
for n=1:nbRepros
  a.currSigma = ones(1,1,nbData) * 10^(2-n);
  %r(n) = reproduction_LQR_finiteHorizon(DataIn, model, a, 0, rFactor);
  r(n) = reproduction_LQR_infiniteHorizon(DataIn, model, a, 0, rFactor);
end

%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[20,50,1300,500]);
hold on; box on;
%Plot target
plot(r(1).Data(1,:), a.currTar, 'r-', 'linewidth', 2);
for n=1:nbRepros
  %Plot trajectories
  plot(r(n).Data(1,:), r(n).Data(2,:), '-', 'linewidth', 2, 'color', ones(3,1)*(n-1)/nbRepros);
end
xlabel('t'); ylabel('x_1');

figure; 
%Plot norm of control commands
subplot(1,2,1); hold on;
for n=1:nbRepros
  plot(DataIn, r(n).ddxNorm, '-', 'linewidth', 2, 'color', ones(3,1)*(n-1)/nbRepros);
end
xlabel('t'); ylabel('|ddx|');
%Plot stiffness
subplot(1,2,2); hold on;
for n=1:nbRepros
  plot(DataIn, r(n).kpDet, '-', 'linewidth', 2, 'color', ones(3,1)*(n-1)/nbRepros);
end
xlabel('t'); ylabel('kp');

%pause;
%close all;
