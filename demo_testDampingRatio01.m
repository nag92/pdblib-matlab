function demo_testDampingRatio01
% Test of the differences between critically damped systems and ideal underdamped systems
%	Sylvain Calinon, 2015

nbData = 100; %Number of datapoints
kp = 200; %Stiffness
kv = 2*kp^.5; %Damping for critically damped system
kv2 = (2*kp)^.5; %Damping for ideal underdamped system
xhat = 1; %Desired target
dt = 0.01; %Time step

%Computation of tracking error for critically damped system
x=0; dx=0; e=0;
for t=1:nbData
	Data(:,t) = [x; dx]; %Log data
	ddx = kp*(xhat-x) - kv * dx; %Spring-damper system
	dx = dx + ddx*dt; %Update velocity
	x = x + dx*dt; %Update position
	e = e + (xhat-x); %Cumulated tracking error
end
disp(['Cumulated tracking error for critically damped system: ' num2str(e)]);

%Computation of tracking error for ideal underdamped system
x=0; dx=0; e=0;
for t=1:nbData
	Data2(:,t) = [x; dx]; %Log data
	ddx = kp*(xhat-x) - kv2 * dx; %Spring-damper system
	dx = dx + ddx*dt; %Update velocity
	x = x + dx*dt; %Update position
	e = e + (xhat-x); %Cumulated tracking error
end
disp(['Cumulated tracking error for ideal underdamped system: ' num2str(e)]);

%Plot
figure; hold on;
plot([1:nbData]*dt, Data(1,:), 'k-');
plot([1:nbData]*dt, Data2(1,:), 'k:');
legend('critically damped', 'ideal underdamped');
plot([1,nbData]*dt, [xhat,xhat], 'r-');
xlabel('t'); ylabel('x');
	
pause;
close all;
	
	
	
	