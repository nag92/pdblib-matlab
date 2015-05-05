function h = plotBimanualRobot(theta,coltmp)
% This function plots the simulated robotic arm in 2D space.
% Sylvain Calinon, 2014

	global r1 r2;

	%Right part
	Rtmp = subs_Rr(theta);
	Ttmp = subs_Tr(theta);
	h = plotBodyBasis(Ttmp(:,1),coltmp);
	h = [h plotBodyT(Rtmp(:,:,1),Ttmp(:,1),r1.L(1),'',coltmp)];
	h = [h plotBodyLink(Rtmp(:,:,2),Ttmp(:,2),r1.L(2),'',coltmp)];
	h = [h plotBodyLink(Rtmp(:,:,3),Ttmp(:,3),r1.L(3),'',coltmp)];

	%Left part
	Rtmp = subs_Rl(theta);
	Ttmp = subs_Tl(theta);
	h = [h plotBodyT(Rtmp(:,:,1),Ttmp(:,1),r2.L(1),'',coltmp)];
	h = [h plotBodyLink(Rtmp(:,:,2),Ttmp(:,2),r2.L(2),'',coltmp)];
	h = [h plotBodyLink(Rtmp(:,:,3),Ttmp(:,3),r2.L(3),'',coltmp)];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function h = plotBodyBasis(p1,coltmp)
% This function plots the basis of the simulated robotic arm in 2D space.
	nbSegm = 50;
	t1 = linspace(0,pi,nbSegm-2);
	xTmp(1,:) = [1.5 1.5*cos(t1) -1.5];
	xTmp(2,:) = [-1.2 1.5*sin(t1) -1.2];
	x = xTmp + repmat(p1,1,nbSegm);
	h = patch(x(1,:),x(2,:),[1 1 1],'edgeColor',coltmp);
	
	xTmp2(1,:) = linspace(-1.2,1.2,5); 
	xTmp2(2,:) = repmat(-1.2,1,5);
	x2 = xTmp2 + repmat(p1,1,5);
	x3 = xTmp2 + repmat(p1+[-.4;-.8],1,5);
	for i=1:5
		h = [h plot([x2(1,i) x3(1,i)],[x2(2,i) x3(2,i)],'-','color',coltmp)];
	end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function h = plotBodyLink(R,T,L,txt,coltmp)
% This function plots an a link of the robotic arm in 2D space.
	nbSegm = 50;
	t1 = linspace(0,-pi,nbSegm/2);
	t2 = linspace(pi,0,nbSegm/2);
	xTmp(1,:) = [sin(t1) L+sin(t2)];
	xTmp(2,:) = [cos(t1) cos(t2)];
	x = R*xTmp + repmat(T,1,nbSegm);
	T2=T+R*[L;0];
	h(1) = fill(x(1,:),x(2,:),[1 1 1],'edgeColor',coltmp);
	h(2) = rectangle('Position',[T(1)-.4,T(2)-.4,.8,.8], 'Curvature',[1,1], 'LineWidth',1,'LineStyle','-','facecolor',[.9 .9 .9],'edgeColor',coltmp);
	h(3) = rectangle('Position',[T2(1)-.4,T2(2)-.4,.8,.8],'Curvature',[1,1], 'LineWidth',1,'LineStyle','-','facecolor',[.9 .9 .9],'edgeColor',coltmp);
	ptxt = R*[L*0.7;0.8] + T;
	h(4) = text(ptxt(1),ptxt(2),txt,'color',coltmp);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function h = plotBodyT(R,T,L,txt,coltmp)
% This function plots an a link of the robotic arm in 2D space.
	nbSegm = 50;
	t1 = linspace(0,-pi,nbSegm/2);
	t2 = linspace(pi,0,nbSegm/2);
	xTmp(1,:) = [sin(t1) L+sin(t2)];
	xTmp(2,:) = [cos(t1) cos(t2)];

	x = R*xTmp + repmat(T,1,nbSegm);
	T2=T+R*[L;0];
	h(1) = fill(x(1,:),x(2,:),[1 1 1],'edgeColor',coltmp);
	h(2) = rectangle('Position',[T(1)-.4,T(2)-.4,.8,.8], 'Curvature',[1,1], 'LineWidth',1,'LineStyle','-','facecolor',[1 1 1],'edgeColor',coltmp);
	ptxt = R*[L*0.7;0.8] + T;
	h(3) = text(ptxt(1),ptxt(2),txt,'color',coltmp);

	T = T2 + R * [0;-L/2];
	R = R * [cos(pi/2) -sin(pi/2); sin(pi/2) cos(pi/2)];
	x = R*xTmp + repmat(T,1,nbSegm);
	T2=T+R*[L;0];
	h(4) = fill(x(1,:),x(2,:),[1 1 1],'edgeColor',coltmp);
	h(5) = rectangle('Position',[T(1)-.4,T(2)-.4,.8,.8], 'Curvature',[1,1], 'LineWidth',1,'LineStyle','-','facecolor',[1 1 1],'edgeColor',coltmp);
	h(6) = rectangle('Position',[T2(1)-.4,T2(2)-.4,.8,.8],'Curvature',[1,1], 'LineWidth',1,'LineStyle','-','facecolor',[1 1 1],'edgeColor',coltmp);
end
