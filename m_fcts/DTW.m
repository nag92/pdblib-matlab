function [x_new, y_new, p] = DTW(x, y, w)
%Trajectory realignment through dynamic time warping
%Sylvain Calinon, 2015

if nargin<3
  w = Inf;
end

nx = size(x,2);
ny = size(y,2);

w = max(w,abs(nx-ny)); 

%Initialization
D = ones(nx+1,ny+1) * Inf; 
D(1,1) = 0;

%DP loop
for i=1:nx
  for j=max(i-w,1):min(i+w,ny)
    D(i+1,j+1) = norm(x(:,i)-y(:,j)) + min([D(i,j+1), D(i+1,j), D(i,j)]);
  end
end

i=nx+1; j=ny+1;
p=[];
while i>1 && j>1
 [~,id] = min([D(i,j-1), D(i-1,j), D(i-1,j-1)]);
 if id==1
   j=j-1;
 elseif id==2
   i=i-1;
 else
   i=i-1;
   j=j-1;
 end
 p = [p [i;j]];
end

p = fliplr(p(:,1:end-1)-1);

x_new = x(:,p(1,:));
y_new = y(:,p(2,:));

%Resampling
x_new = spline(1:size(x_new,2), x_new, linspace(1,size(x_new,2),nx));
y_new = spline(1:size(y_new,2), y_new, linspace(1,size(y_new,2),nx));




 
