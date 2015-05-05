function h = plotArmBasis(p1, sz, facecol, edgecol)
%Sylvain Calinon, 2014

nbSegm = 30;
sz = sz*1.2;

t1 = linspace(0,pi,nbSegm-2);
xTmp(1,:) = [sz*1.5 sz.*1.5*cos(t1) -sz*1.5];
xTmp(2,:) = [-sz*1.2 sz.*1.5*sin(t1) -sz*1.2];
xTmp(3,:) = [t1(1) t1 t1(end)];
x = xTmp + repmat(p1,1,nbSegm);
h = patch(x(1,:),x(2,:), facecol,'edgecolor',edgecol); %,'edgealpha',.9,'facealpha',.9

xTmp2(1,:) = linspace(-sz*1.2,sz*1.2,5);
xTmp2(2,:) = repmat(-sz*1.2,1,5);
xTmp2(3,:) = zeros(1,5);
x2 = xTmp2 + repmat(p1,1,5);
x3 = xTmp2 + repmat(p1+[-0.5;-1;0]*sz,1,5);
for i=1:5
	h = [h patch([x2(1,i) x3(1,i)], [x2(2,i) x3(2,i)], facecol,'edgecolor',facecol)]; %,'edgealpha',.9,'facealpha',.9
end
