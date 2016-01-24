clear all;
close all;
clc;

% Read csv file
data = csvread('csvlist.dat');
x = data(:,2);  % w1 joint position
x = x-x(1);     % Scale so that start point is 0
t = data(:,1);  % t = timestamp

% Set DMP parameters
dt = 0.01;          % Position record rate
goal = x(end);      % Goal location i.e. where the DMP will converge
tau = t(end)-t(1);  % Time constant (roughly equal to movement time until convergence)
n_rfs = 10;         % Number of basis functions
ID = 1;             % DMP ID

% initialize the motor primitive and storage variables
dcp('clear',ID);                        % Clear any previous DMPs
dcp('init',ID,n_rfs,'w1_joint',1);      % Initialise DMP and name it

rt = 0:dt:tau;                      % Discrete timesteps
y = zeros(1,length(rt));            % Position
yd = zeros(1,length(rt));           % Velocity
ydd = zeros(1,length(rt));          % Acceleration
basis = zeros(n_rfs,length(rt));    % Basis functions

% Fit DMP to target trajectory
dcp('Batch_Fit',ID,tau,dt,x);

% reset the motor primitive
dcp('reset_state',ID);
dcp('set_goal',ID,goal,1);

% run the motor primitive & precalculate the basis functions
for i=1:length(rt)
	% also store the values of the basis functions
    [y(i),yd(i),ydd(i),basis(:,i)] = dcp('run',ID,tau,dt);
end

% Plot DMP trajectories and ask user to verify
subplot(1,3,1)
plot(y);
hold on; plot(x); hold off;
subplot(1,3,2)
plot(yd);
subplot(1,3,3)
plot(ydd);
disp('Check DMP trajectories, press enter to continue')
pause
%---------------------PoWER Stuff---------------------------------------%

