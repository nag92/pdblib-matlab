function [p2, h] = plotArmLink(a1, d1, p1, sz, facecol, edgecol)
%Sylvain Calinon, 2014

nbSegm = 30;

p1 = p1 + [0; 0; .1];
t1 = linspace(0,-pi,nbSegm/2);
t2 = linspace(pi,0,nbSegm/2);
xTmp(1,:) = [sz.*sin(t1) d1+sz.*sin(t2)];
xTmp(2,:) = [sz.*cos(t1) sz.*cos(t2)];
xTmp(3,:) = zeros(1,nbSegm);
R = [cos(a1) -sin(a1) 0; sin(a1) cos(a1) 0; 0 0 0];
x = R*xTmp + repmat(p1,1,nbSegm);
p2 = R*[d1;0;0] + p1;
h = patch(x(1,:),x(2,:),x(3,:),facecol,'edgecolor',edgecol); %,'linewidth',1,,'facealpha',.9,'edgealpha',.9
msh = [sin(linspace(0,2*pi,nbSegm)); cos(linspace(0,2*pi,nbSegm))]*sz*0.2;
h = [h patch(msh(1,:)+p1(1), msh(2,:)+p1(2), repmat(p1(3),1,nbSegm), edgecol,'edgeColor',edgecol)]; %,'facealpha',.9,'edgealpha',.9
h = [h patch(msh(1,:)+p2(1), msh(2,:)+p2(2), repmat(p2(3),1,nbSegm), edgecol,'edgeColor',edgecol)]; %,'facealpha',.9,'edgealpha',.9

