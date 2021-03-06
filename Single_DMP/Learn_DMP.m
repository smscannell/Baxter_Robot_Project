clear all;
close all;
clc;
global dcps
%----------------------------DMP stuff------------------------------------%

% Read csv file containing demonstration trajectory
[tau,x] = file_parse(1,7);

% Set DMP parameters
x_init = x(1);
x = x-x_init;       % Scale position values
dt = 0.01;          % step size (equal to sampling rate of joint trajectory)
goal = x(end);      % Goal location i.e. where the DMP will converge
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
% Set initial trajectory
y1=y; yd1=yd; ydd1=ydd;

% Plot DMP trajectories and ask user to verify
subplot(1,3,1)
plot(rt,y1); xlabel('Time (seconds)'); ylabel('Position (radians)');
hold on; plot(rt,x); hold off;
subplot(1,3,2)
plot(rt,yd1); xlabel('Time (seconds)'); ylabel('Velocity (radians/s)');
subplot(1,3,3)
plot(rt,ydd1); xlabel('Time (seconds)'); ylabel('Acceleration (radians/s^2)');
disp('Check DMP trajectories, press enter to continue')
pause
close all
%---------------------PoWER Stuff---------------------------------------%
% The required motor primitive code can be downloaded from
% http://www-clmc.usc.edu/Resources/Software
iter=1;
param(:,1) = dcps(1).w;     % set initial parameters
current_param = param(:,1);     % Set current parameters to initial
% set the initial variance
variance(:,1) = 4000.*ones(n_rfs,1);
variance(:,2) = 4000.*ones(n_rfs,1);
% Set up imporatance samping table
s_Return = [0 0];

% Main loop
while 0==0
    % Write DMP trajectory to csv file. Note: L and R gripper signals set
    % to 0 and 100 respectively.
    M=[rt' (x_init+y)' zeros(length(rt),1) 100*ones(length(rt),1)]; 
    fid=fopen('DMP_out','w'); fprintf(fid, '%s,','time','left_w1','left_gripper');
    fprintf(fid, '%s\n','right_gripper'); fclose(fid);
    dlmwrite('DMP_out',M,'-append')
    % Get error from user input
    Error(iter) = input('Enter ditsance of ball from target in mm ');
    % Calculate return based on error
    Return(iter) = exp(-(abs(Error(iter))));
    
    % add current return to importance samping table and sort by increasing
    % return
    s_Return = cat(1,s_Return,[Return(iter) iter]);
    s_Return = sortrows(s_Return);
    
    % reset policy parameters to 0
    param_nom = zeros(n_rfs,1);
    param_dnom = zeros(n_rfs,1);
    
    % calculate the expectations (the normalization is taken care of by the division)
    % as importance sampling we take the 10 best rollouts
    for i=1:min(iter,10)
        % get the rollout number for the 10 best rollouts
        j = s_Return(end+1-i,2);
		% calculate weighting
        temp_W = variance(:,j).^-1;
		% calculate the exploration with respect to the current parameters
        temp_explore = (param(:,j)-current_param);
		% as we use the return, and assume that always only one basis 
        % functions is active we get these simple sums
        param_nom = param_nom + temp_W.*temp_explore*Return(j);
        param_dnom = param_dnom + temp_W.*Return(j);
    end
    
     % update the parameters
    param(:,iter+1) = current_param + param_nom./(param_dnom+1.e-10);
    
	% update the variances
    var_nom = zeros(n_rfs,1);
    var_dnom = 0;
    if iter>1
		% we use more rollouts for the variance calculation to avoid 
		% rapid convergence to 0
        for i=1:min(iter,100)
			% get the rollout number for the 100 best rollouts
            j = s_Return(end+1-i,2);
			% calculate the exploration with respect to the current parameters
            temp_explore = (param(:,j)-current_param);
            % Weight exploration with the associated return
            var_nom = var_nom + temp_explore.^2.*Return(j);
            % Normalise
            var_dnom = var_dnom + Return(j);
        end
		% apply an upper and a lower limit to the exploration
        variance(:,iter+1) = max(var_nom./(var_dnom+1.e-10),.1.*variance(:,1));
        variance(:,iter+1) = min(variance(:,iter+1),10.*variance(:,1));
    end
    
    current_param = param(:,iter+1);
    
    cont = input('Continue? (y/n): ','s');
    if cont == 'y'
        % Add exploration to the new parameters
        param(:,iter+1) = param(:,iter+1) + variance(:,iter+1).^.5.*randn(n_rfs,1);

        % apply the new parameters to the motor primitve
        dcp('change',ID,'w',param(:,iter+1));
    
        % reset the motor primitive
        dcp('reset_state',ID);
        dcp('set_goal',ID,goal,1);
    
        % run the motor primitive
        for i=1:length(rt)
            [y(i),yd(i),ydd(i)] = dcp('run',ID,tau,dt);
        end
    else
        % apply the new parameters to the motor primitve
        dcp('change',ID,'w',param(:,iter+1));
    
        % reset the motor primitive
        dcp('reset_state',ID);
        dcp('set_goal',ID,goal,1);
    
        % run the motor primitive
        for i=1:length(rt)
            [y(i),yd(i),ydd(i)] = dcp('run',ID,tau,dt);
        end
        break
    end
    
    % Plot new trajectory
    subplot(1,3,1)
    plot(y);
    hold on; plot(y1); hold off;
    subplot(1,3,2)
    plot(yd);
    subplot(1,3,3)
    plot(ydd);

    iter = iter + 1;
end

% Plot error values
figure; subplot(1,2,1);
plot(y);
hold on; plot(y1); hold off;
subplot(1,2,2);
plot(Error)
coeffs = polyfit(1:1:length(Error),Error,1);
fittedX = linspace(1,length(Error),299);
fittedY = polyval(coeffs, fittedX);
hold on; plot(fittedX, fittedY, 'r-', 'Linewidth', 1.5); hold off;