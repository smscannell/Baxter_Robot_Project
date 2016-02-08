clear all;
close all;
%clc;
global dcps
%----------------------------DMP stuff------------------------------------%

% Read csv file containing w1 joint trajectory
[tau,x_w1,x_e1] = file_parse_multi(1,7,5);

% Set universal parameters
dt = 0.01;        % step size (equal to sampling rate of joint trajectory)
rt = 0:dt:tau;    % Discrete timesteps

% Set DMP parameters for w1
x_init_w1 = x_w1(1);
x_w1 = x_w1-x_init_w1;       % Scale position values
goal_w1 = x_w1(end);         % Goal location i.e. where the DMP will converge     
n_rfs_w1 = 10;               % Number of basis functions
% Set DMP parameters for e1
x_init_e1 = x_e1(1);
x_e1 = x_e1-x_init_e1;       % Scale position values
goal_e1 = x_e1(end);         % Goal location i.e. where the DMP will converge
n_rfs_e1 = 10;               % Number of basis functions


% initialize the motor primitive and storage variables for w1
dcp('clear',1);                           % Clear any previous DMPs
dcp('init',1,n_rfs_w1,'w1_joint',1);      % Initialise DMP (ID,nrfs,name,flag)
% initialize the motor primitive and storage variables for E1
dcp('clear',2);                           % Clear any previous DMPs
dcp('init',2,n_rfs_e1,'e1_joint',1);      % Initialise DMP (ID,nrfs,name,flag)


% Fit w1 DMP to target trajectory
dcp('Batch_Fit',1,tau,dt,x_w1);
% Fit e1 DMP to target trajectory
dcp('Batch_Fit',2,tau,dt,x_e1);


% Initialise DMP and check for w1
[y_w1,yd_w1,ydd_w1,basis_w1]=DMP_init(1,goal_w1,tau,rt,dt,x_w1);
% Set initial trajectory for w1
y1_w1=y_w1; yd1_w1=yd_w1; ydd1_w1=ydd_w1;
% Initialise DMP and check for e1
[y_e1,yd_e1,ydd_e1,basis_e1]=DMP_init(2,goal_e1,tau,rt,dt,x_e1);
% Set initial trajectory for w1
y1_e1=y_e1; yd1_e1=yd_e1; ydd1_e1=ydd_e1;


%---------------------PoWER Stuff---------------------------------------%
% The required motor primitive code can be downloaded from
% http://www-clmc.usc.edu/Resources/Software
iter=1;
param(:,1) = [dcps(1).w; dcps(2).w];     % set initial parameters
current_param = param(:,1);     % Set current parameters to initial
% set the initial variance
variance(:,1) = 4000.*ones(n_rfs_w1+n_rfs_e1,1);
variance(:,2) = 4000.*ones(n_rfs_w1+n_rfs_e1,1);
% Set up imporatance samping table
s_Return = [0 0];

% Main loop
while 0==0
    % Write DMP trajectory to csv file. Note: L and R gripper signals set
    % to 0 and 100 respectively.
    M=[rt' (x_init_e1+y_e1)' (x_init_w1+y_w1)' zeros(length(rt),1) 100*ones(length(rt),1)]; 
    fid=fopen('DMP_out','w'); fprintf(fid, '%s,','time','left_e1','left_w1','left_gripper');
    fprintf(fid, '%s\n','right_gripper'); fclose(fid);
    dlmwrite('DMP_out',M,'-append')
    % Get error from user input
    Error(iter) = input('Enter ditsance of ball from target in mm ');
    close all
    % Calculate return based on error
    Return(iter) = exp(-(abs(Error(iter))));
    
    % add current return to importance sampling table and sort by increasing
    % return
    s_Return = cat(1,s_Return,[Return(iter) iter]);
    s_Return = sortrows(s_Return);
    
    % reset policy parameters to 0
    param_nom = zeros(n_rfs_w1+n_rfs_e1,1);
    param_dnom = zeros(n_rfs_w1+n_rfs_e1,1);
    
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
    var_nom = zeros(n_rfs_w1+n_rfs_e1,1);
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
        param(:,iter+1) = param(:,iter+1) + variance(:,iter+1).^.5.*randn(n_rfs_w1+n_rfs_e1,1);

        % apply the new parameters to the w1 motor primitve
        dcp('change',1,'w',param(1:n_rfs_w1,iter+1));
        % apply the new parameters to the e1 motor primitve
        dcp('change',2,'w',param(n_rfs_w1+1:n_rfs_w1+n_rfs_e1,iter+1));
    
        % reset the w1 motor primitive
        dcp('reset_state',1);
        dcp('set_goal',1,goal_w1,1);
        % reset the e1 motor primitive
        dcp('reset_state',2);
        dcp('set_goal',2,goal_e1,1);
    
        % run the w1 motor primitive
        for i=1:length(rt)
            [y_w1(i),yd_w1(i),ydd_w1(i)] = dcp('run',1,tau,dt);
        end
        % run the e1 motor primitive
        for i=1:length(rt)
            [y_e1(i),yd_e1(i),ydd_e1(i)] = dcp('run',2,tau,dt);
        end
    else
        % apply the new parameters to the w1 motor primitve
        dcp('change',1,'w',param(1:n_rfs_w1,iter+1));
        % apply the new parameters to the e1 motor primitve
        dcp('change',2,'w',param(n_rfs_w1+1:n_rfs_w1+n_rfs_e1,iter+1));
    
        % reset the w1 motor primitive
        dcp('reset_state',1);
        dcp('set_goal',1,goal_w1,1);
        % reset the e1 motor primitive
        dcp('reset_state',1);
        dcp('set_goal',1,goal_e1,1);
    
        % run the w1 motor primitive
        for i=1:length(rt)
            [y_w1(i),yd_w1(i),ydd_w1(i)] = dcp('run',1,tau,dt);
        end
        % run the e1 motor primitive
        for i=1:length(rt)
            [y_e1(i),yd_e1(i),ydd_e1(i)] = dcp('run',2,tau,dt);
        end
        break
    end
    
    % Plot new w1 trajectory
    figure('Name','w1 Joint Trajectory')
    subplot(1,3,1)
    plot(y_w1); title('Position');
    hold on; plot(y1_w1); hold off;
    subplot(1,3,2)
    plot(yd_w1); title('Velocity');
    subplot(1,3,3)
    plot(ydd_w1); title('Acceleration');
    % Plot new e1 trajectory
    figure('Name','e1 Joint Trajectory')
    subplot(1,3,1)
    plot(y_e1); title('Position');
    hold on; plot(y1_e1); hold off;
    subplot(1,3,2)
    plot(yd_e1); title('Velocity');
    subplot(1,3,3)
    plot(ydd_e1); title('Acceleration');

    iter = iter + 1;
end

% Plot error values
figure; subplot(1,3,1);
plot(y_w1);
hold on; plot(y1_w1); hold off;
subplot(1,3,2);
plot(y_e1);
hold on; plot(y1_e1); hold off;
subplot(1,3,3);
plot(Error)
coeffs = polyfit(1:1:length(Error),Error,1);
fittedX = linspace(1,length(Error),299);
fittedY = polyval(coeffs, fittedX);
hold on; plot(fittedX, fittedY, 'r-', 'Linewidth', 1.5); hold off;