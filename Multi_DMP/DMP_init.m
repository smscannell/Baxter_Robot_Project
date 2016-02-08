function [ y,yd,ydd,basis ] = DMP_init( ID,goal,tau,rt,dt,x )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% reset the motor primitive
dcp('reset_state',ID);
% set goal
dcp('set_goal',ID,goal,1);

% run the motor primitive & precalculate the basis functions
for i=1:length(rt)
	% also store the values of the basis functions
    [y(i),yd(i),ydd(i),basis(:,i)] = dcp('run',ID,tau,dt);
end

% Plot DMP trajectories and ask user to verify
subplot(1,3,1)
plot(rt,y); xlabel('Time (seconds)'); ylabel('Position (radians)');
hold on; plot(rt,x); hold off;
subplot(1,3,2)
plot(rt,yd); xlabel('Time (seconds)'); ylabel('Velocity (radians/s)');
subplot(1,3,3)
plot(rt,ydd); xlabel('Time (seconds)'); ylabel('Acceleration (radians/s^2)');
disp('Check DMP trajectories, press enter to continue')
pause
close all

end

