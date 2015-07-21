function demoIK_nullspace_TPGMM01
% Inverse kinematics with nullspace treated with task-parameterized GMM (bimanual tracking task, version with 4 frames).
%
% This example requires Peter Corke's robotics toolbox (run 'startup_rvc' from the robotics toolbox).
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
model.nbStates = 1; %Number of states
model.nbFrames = 4; %Number of frames
model.nbVars = [2,2,2,2]; %[xl],[xr2],[xl2],[xr]
model.nbVar = max(model.nbVars);
model.nbQ = 5; %Number of variables in configuration space (joint angles)
model.nbX = 2; %Number of variables in operational space (end-effector position)
model.nbVarOut = model.nbQ; %[q]
model.dt = 0.01; %Time step
nbSamples = 1; %Number of demonstration
nbRepros = 1; %Number of reproduction
nbData = 200; %Number of datapoints in a demonstration
pinvDampCoeff = 1e-8; %Coefficient for damped pseudoinverse

needsModel = 1;


%% Create robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
armLength = .5;
L1 = Link('d', 0, 'a', armLength, 'alpha', 0);
arm = SerialLink(repmat(L1,3,1));


%% Generate demonstrations 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if needsModel==1
disp('Demonstration...');
for n=1:nbSamples
	q = [pi/2 pi/2 pi/3 -pi/2 -pi/3]'; %Initial pose
	for t=1:nbData
		s(n).q(:,t) = q; %Log data
		%Forward kinematics
		Htmp = arm.fkine(q(1:3));
		s(n).lx(:,t) = Htmp(1:2,end);
		Htmp = arm.fkine(q([1,4:5]));
		s(n).rx(:,t) = Htmp(1:2,end);
		%Reference trajectory
		if t==1 
			%Objects moving on a line
			s(n).lxh = [linspace(s(n).lx(1,1),s(n).lx(1,1)-.6*armLength,nbData); linspace(s(n).lx(2,1),s(n).lx(2,1)+2*armLength,nbData)];
			s(n).rxh = [linspace(s(n).rx(1,1),s(n).rx(1,1)+.6*armLength,nbData); linspace(s(n).rx(2,1),s(n).rx(2,1)+2*armLength,nbData)];
% 			%Objects moving on a curve
% 			s(n).lxh = [-sin(linspace(0,pi,nbData))*0.4+s(n).lx(1,1); linspace(s(n).lx(2,1),s(n).lx(2,1)+2*armLength,nbData)];
% 			s(n).rxh = [sin(linspace(0,pi,nbData))*0.4+s(n).rx(1,1); linspace(s(n).rx(2,1),s(n).rx(2,1)+2*armLength,nbData)];
		end
		%Build Jacobians
		lJ = arm.jacob0(q(1:3),'trans');
		lJ = lJ(1:2,:);
		rJ = arm.jacob0(q([1,4:5]),'trans');
		rJ = rJ(1:2,:);
		J = lJ; 
		J(3:4,[1,4:5]) = rJ;
		Ja = J(1:2,:);
		Jb = J(3:4,:);
		pinvJ = (J'*J+eye(model.nbQ)*pinvDampCoeff) \ J'; %damped pseudoinverse
		pinvJa = (Ja'*Ja+eye(model.nbQ)*pinvDampCoeff) \ Ja'; %damped pseudoinverse
		pinvJb = (Jb'*Jb+eye(model.nbQ)*pinvDampCoeff) \ Jb'; %damped pseudoinverse
		
% 		Na = eye(model.nbQ) - pinvJa*Ja; %Nullspace projection matrix
% 		Nb = eye(model.nbQ) - pinvJb*Jb; %Nullspace projection matrix
		
		%An alternative way of computing the nullspace projection matrix is given by
		%http://math.stackexchange.com/questions/421813/projection-matrix-onto-null-space
		[U,S,V] = svd(pinvJa);
		S2 = zeros(model.nbQ);
		S2(model.nbX+1:end,model.nbX+1:end) = eye(model.nbQ-model.nbX);
		Na = U * S2 * U';
% 		%pinvNa = U*(eye(5)-S2)*U';
% 		pinvNa = U*pinv(S2)*U';
		
		[U,S,V] = svd(pinvJb);
		S2 = zeros(model.nbQ);
		S2(model.nbX+1:end,model.nbX+1:end) = eye(model.nbQ-model.nbX);
		Nb = U * S2 * U';
% 		%pinvNb = U*(eye(5)-S2)*U';
% 		pinvNb = U*pinv(S2)*U';
		
		%IK controller
		ldx = (s(n).lxh(:,t) - s(n).lx(:,t)) / model.dt;
		rdx = (s(n).rxh(:,t) - s(n).rx(:,t)) / model.dt;
		
		%Generate artificial dataset
		%dq =  pinvJ * [ldx; rdx]; %Equal priority between arms
		dq =  pinvJa * ldx + Na * pinvJb * rdx;	%Priority on left arm
		%dq =  pinvJb * rdx + Nb * pinvJa * ldx; %Priority on right arm
		
		s(n).fr(1).Data(:,t) = ldx * model.dt;
		s(n).fr(2).Data(:,t) = Jb*Na*pinvJb*rdx * model.dt;
		s(n).fr(3).Data(:,t) = Ja*Nb*pinvJa*ldx * model.dt;
		s(n).fr(4).Data(:,t) = rdx * model.dt;
		
		q = q + dq * model.dt;
	end
end


%% Create dataset
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Data = zeros(model.nbVar, model.nbFrames, nbData*nbSamples);
for n=1:nbSamples
	s(n).nbData = nbData;
	for m=1:model.nbFrames
		Data(1:model.nbVars(m),m,(n-1)*nbData+1:n*nbData) = s(n).fr(m).Data; 
	end
end


%% TP-GMM learning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('Parameters estimation of TP-GMM with EM:');
model = init_TPGMM_timeBased(Data, model); %Initialization
%model = init_TPGMM_kmeans(Data, model); %Initialization
model = EM_TPGMM(Data, model);

model.nbVar = model.nbQ; %Update of nbVar to later use productTPGMM()


%% Reproduction 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Reproduction...');
rr.Priors = model.Priors;
rr.nbStates = model.nbStates;
for n=1:nbRepros
	r(n).q(:,1) = [pi/2 pi/2 pi/3 -pi/2 -pi/3]; %Initial pose
	for t=1:nbData
		%Forward kinematics
		Htmp = arm.fkine(r(n).q(1:3,t));
		r(n).lx(:,t) = Htmp(1:2,end);
		Htmp = arm.fkine(r(n).q([1,4:5],t));
		r(n).rx(:,t) = Htmp(1:2,end);
		%Reference trajectory
		if t==1
			%Objects moving on a line
			r(n).lxh = [linspace(r(n).lx(1,1),r(n).lx(1,1)-.6*armLength,nbData); linspace(r(n).lx(2,1),r(n).lx(2,1)+2*armLength,nbData)];
			r(n).rxh = [linspace(r(n).rx(1,1),r(n).rx(1,1)+.6*armLength,nbData); linspace(r(n).rx(2,1),r(n).rx(2,1)+2*armLength,nbData)];
% 			%Objects moving on a curve
% 			r(n).lxh = [-sin(linspace(0,pi,nbData))*0.3+r(n).lx(1,1); linspace(r(n).lx(2,1),r(n).lx(2,1)+2*armLength,nbData)];
% 			r(n).rxh = [sin(linspace(0,pi,nbData))*0.3+r(n).rx(1,1); linspace(r(n).rx(2,1),r(n).rx(2,1)+2*armLength,nbData)];
		end
		%IK controller
		ldx = (r(n).lxh(:,t) - r(n).lx(:,t)) / model.dt;
		rdx = (r(n).rxh(:,t) - r(n).rx(:,t)) / model.dt;
		%Build Jacobians
		lJ = arm.jacob0(r(n).q(1:3,t),'trans');
		lJ = lJ(1:2,:);
		rJ = arm.jacob0(r(n).q([1,4:5],t),'trans');
		rJ = rJ(1:2,:);
		J = lJ; 
		J(3:4,[1,4:5]) = rJ;
		Ja = J(1:2,:);
		Jb = J(3:4,:);
		pinvJa = (Ja'*Ja+eye(model.nbQ)*pinvDampCoeff) \ Ja'; %damped pseudoinverse
		pinvJb = (Jb'*Jb+eye(model.nbQ)*pinvDampCoeff) \ Jb'; %damped pseudoinverse
				
% 		Na = eye(model.nbQ) - pinvJa*Ja; %Nullspace projection matrix
% 		Nb = eye(model.nbQ) - pinvJb*Jb; %Nullspace projection matrix
		
		%An alternative way of computing the nullspace projection matrix is given by
		%http://math.stackexchange.com/questions/421813/projection-matrix-onto-null-space
		[U,S,V] = svd(pinvJa);
		S2 = zeros(model.nbQ);
		S2(model.nbX+1:end,model.nbX+1:end) = eye(model.nbQ-model.nbX);
		Na = U * S2 * U';
% 		%pinvNa = U*(eye(5)-S2)*U';
% 		pinvNa = U*pinv(S2)*U';
		
		[U,S,V] = svd(pinvJb);
		S2 = zeros(model.nbQ);
		S2(model.nbX+1:end,model.nbX+1:end) = eye(model.nbQ-model.nbX);
		Nb = U * S2 * U';
% 		%pinvNb = U*(eye(5)-S2)*U';
% 		pinvNb = U*pinv(S2)*U';
		
		%Update frames
		%Priority on left arm (dq =  pinvJa * ldx + Na * pinvJb * rdx)
		%left
		pTmp(1).A = pinvJa;
		pTmp(1).b = r(n).q(:,t) + pinvJa * ldx * model.dt;
		%right
		pTmp(2).A = Na * pinvJb; 
		pTmp(2).b = r(n).q(:,t) + Na * pinvJb * rdx * model.dt; 
		%Priority on right arm (dq =  pinvJb * rdx + Nb * pinvJa * ldx)
		%left
		pTmp(3).A = Nb * pinvJa; 
		pTmp(3).b = r(n).q(:,t) + Nb * pinvJa * ldx * model.dt; 
		%right
		pTmp(4).A = pinvJb;
		pTmp(4).b = r(n).q(:,t) + pinvJb * rdx * model.dt; 
		
		%Reproduction with TPGMM
		[rr.Mu, rr.Sigma] = productTPGMM(model, pTmp);
		r(n).q(:,t+1) = rr.Mu; 
	end
end

save('data/TPGMMtmp.mat','s','r','model');
end %needsModel
load('data/TPGMMtmp.mat');


%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10,10,1000,450],'color',[1 1 1]); 
colTmp = repmat(linspace(.8,.2,nbData),3,1)';

%DEMOS
subplot(1,2,1); hold on; axis off; title('Demonstration');
for n=1:nbSamples
	for t=round(linspace(1,nbData,10))	
		plotArm(s(n).q(1:3,t), ones(3,1)*armLength, [0;0;t/nbData], .02, colTmp(t,:)); %left arm
		plotArm(s(n).q([1,4:5],t), ones(3,1)*armLength, [0;0;t/nbData], .02, colTmp(t,:)); %right arm
	end
end
for n=1:nbSamples
	plot3(s(n).rxh(1,:), s(n).rxh(2,:), ones(1,nbData)*2, 'r-','linewidth',2);
	plot3(s(n).lxh(1,:), s(n).lxh(2,:), ones(1,nbData)*2, 'r-','linewidth',2);
end
set(gca,'xtick',[],'ytick',[]); axis equal; %axis([-1.1 1.1 -.1 1.2]); 

%REPROS
subplot(1,2,2); hold on; axis off; title('Reproduction');
for n=1:nbRepros
	for t=round(linspace(1,nbData,10))
		plotArm(r(n).q(1:3,t), ones(3,1)*armLength, [0;0;t/nbData], .02, colTmp(t,:)); %left arm
		plotArm(r(n).q([1,4:5],t), ones(3,1)*armLength, [0;0;t/nbData], .02, colTmp(t,:)); %right arm
	end
end
for n=1:nbRepros
	plot3(r(n).rxh(1,:), r(n).rxh(2,:), ones(1,nbData)*2, 'r-','linewidth',2);
	plot3(r(n).lxh(1,:), r(n).lxh(2,:), ones(1,nbData)*2, 'r-','linewidth',2);
end
set(gca,'xtick',[],'ytick',[]); axis equal; 

%print('-dpng','graphs/demoIK_nullspace_TPGMM01.png');
%pause;
%close all;

