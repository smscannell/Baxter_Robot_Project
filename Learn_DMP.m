clear
clc
global dcps

% Read csv file
data = csvread('csvlist.dat',1,0);
x = data(:,3);  % w1 joint position
t = data(:,1);  % t = timestamp

% Set DMP parameters
dt = 0.01;          % Position record rate
goal = x(end);      % Goal location i.e. where the DMP will converge
tau = t(end)-t(1);  % Time constant (roughly equal to movement time until convergence)
n_rfs = 10;         % Number of basis functions
ID = 1;             % DMP ID

% initialize DMP
dcp('clear',ID);                        % Clear any previous DMPs
dcp('init',ID,n_rfs,'w1_joint',1);      % Initialise DMP and name it




