function h = plotArm(a, d, p, sz, facecolor, edgecolor)
%Sylvain Calinon, 2014

if nargin<4
	sz = .05;
end
if nargin<5
	facecolor = [.5,.5,.5];
end
if nargin<6
	edgecolor = [1,1,1];
end
if size(p,1)==2
	p = [p; -1];
end

h = plotArmBasis(p, sz, facecolor, edgecolor);
for i=1:length(a)
	[p, hTmp] = plotArmLink(sum(a(1:i)), d(i), p, sz, facecolor, edgecolor);
	h = [h hTmp];
end
