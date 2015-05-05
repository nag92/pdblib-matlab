function h = plot2DArrow(pos,dir,col)
%Sylvain Calinon, 2015

h(1) = plot([pos(1) pos(1)+dir(1)], [pos(2) pos(2)+dir(2)], '-','linewidth',2,'color',col);
sz = 8E-2;
pos = pos+dir;
if norm(dir)>sz
  dir = dir/norm(dir);
  prp = [dir(2); -dir(1)];
  dir = dir*sz;
  prp = prp*sz;
  msh = [pos pos-dir-prp/2 pos-dir+prp/2 pos];
  h(2) = patch(msh(1,:),msh(2,:),col,'edgecolor',col);
end