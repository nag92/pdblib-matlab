function demo_DTW01
% Trajectory realignment through dynamic time warping (DTW).
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nbData = 200; %Length of each trajectory
wMax = 50; %Warping time window 
nbSamples = 5; %Number of demonstrations
nbVar = 2; %Number of dimensions (max 2 for AMARSI data)


%% Load AMARSI handwriting data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
demos=[];
load('data/AMARSI/GShape.mat');
for n=1:nbSamples
	s(n).Data = spline(1:size(demos{n}.pos,2), demos{n}.pos(1:nbVar,:), linspace(1,size(demos{n}.pos,2),nbData)); %Resampling
end


%% Dynamic time warping
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
r(1).Data = s(1).Data;
for n=2:nbSamples
	[r(1).Data, r(n).Data, r(n-1).wPath] = DTW(r(1).Data, s(n).Data, wMax);
	%Realign previous trajectories
	p = r(n-1).wPath(1,:);
	for m=2:n-1
		DataTmp = r(m).Data(:,p);
		r(m).Data = spline(1:size(DataTmp,2), DataTmp, linspace(1,size(DataTmp,2),nbData)); %Resampling
	end
end


%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10,10,1000,500]); 
for k=1:nbVar
	subplot(2,nbVar,(k-1)*2+1); hold on; if k==1 title('Before DTW'); end;
	for n=1:nbSamples
		plot(s(n).Data(k,:), '-','linewidth',1,'color',[0 0 0]);
	end
	xlabel('t'); ylabel(['x_' num2str(k)]);
	axis tight; set(gca,'Xtick',[]); set(gca,'Ytick',[]);
	subplot(2,nbVar,(k-1)*2+2); hold on; if k==1 title('After DTW'); end;
	for n=1:nbSamples
		plot(r(n).Data(k,:), '-','linewidth',1,'color',[0 0 0]);
	end
	xlabel('t'); ylabel(['x_' num2str(k)]);
	axis tight; set(gca,'Xtick',[]); set(gca,'Ytick',[]);
end

figure; hold on;
for n=1:nbSamples-1
	plot(r(n).wPath(1,:),r(n).wPath(2,:),'-','color',[0 0 0]);
end
xlabel('w_1'); ylabel('w_2');

%print('-dpng','graphs/demo_DTW01.png');
%pause;
%close all;
