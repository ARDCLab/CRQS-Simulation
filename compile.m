% Title: compile.m
% Author(s): William Elke III
% Date: 12-Jan-2021
% Description: Script with the purpose of simulating the nonlinear dynamics
% of the Cost and Risk-reducing Quadcopter System (CRQS).
%
% Updated 02-Jul-2023
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
clc
close all

% Add paths to necessary scripts
maindir = [pwd,'\'];
addpath([maindir,'files_for_control'],...
    [maindir,'files_for_dynamics'],[maindir,'files_for_support'],...
    [maindir,'files_for_trajectory'],[maindir,'results']);

% Select version:
% % version = 1: 
% % version = 2:
version = 2;

% Select CRQS configuration. All configuration include the quadcopter plus
% the specified features. Must match exactly.
% % config_name = 'RIP': Rigid inverted pendulum (RIP)
% % config_name = 'RIP+HP': RIP and hanging pendulum (HP)
% % config_name = 'FIP': Flexible inverted pendulum (FIP)
% % config_name = 'FIP+HP': FIP and HP
config_name = 'FIP+HP';

% Version Name
if ~exist(['.\results\version',num2str(version),'\',config_name])
    mkdir(['.\results\version',num2str(version),'\',config_name])
end

disp('Initializing...')

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Constants
constants

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Load Trajectory Data
data = load([maindir,'files_for_trajectory\version2\trajectory_data.mat']);
t_extra_start = 2; % Hover time added to the beginning of the simulation
t_extra_end = 8; % Hover time added to the end of the simulation
t_max = data.t_traj(end) + t_extra_start + t_extra_end; % [s] Total simulation time
dT_traj = data.t_traj(2) - data.t_traj(1); % [s] Trajectory time step size
t_traj = 0:dT_traj:t_max;
u_d = const.bar_u.*ones(4,length(t_traj)); % Feedforward input
x_d = [[data.x_d(1:3,1).*ones(3,t_extra_start/dT_traj); zeros(13,t_extra_start/dT_traj)], data.x_d, [data.x_d(1:3,end).*ones(3,t_extra_end/dT_traj); zeros(13,t_extra_end/dT_traj)]];
% x_d = data.x_d(:,1).*ones(16,length(t_traj)); % Hover
clear data

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation time
t0 = t_traj(1); % [s] Start time
dT_CL = 1/50; % [s] Time-step size the closed-loop is running at. t_max/dT_CL must be an integer.
t_span_CL = t0:dT_CL:t_max;
t_div_CL = length(t_span_CL); % Number of time steps the inner loop completes during the simulation
Nsteps = 10; % Number of continuous-time steps between discrete-time steps. For storing results.
t = round(linspace(t0,t_max,Nsteps*((t_max/dT_CL)+1)),3); % Time variable for storing results.
dT_dyn = t(2) - t(1); % [s] Step size for storing results
t_div_sim = length(t); % Number of time steps for continuous-time results storage.

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Load Controller Data
if version == 1
    data = load([maindir,'files_for_control\control_params_1.mat']);
elseif version == 2
    data = load([maindir,'files_for_control\control_params_2.mat']);
else
    disp('Version error.')
end    
Kr = data.Kr; % Discrete LQR constant-gain matrix
const.size_u = size(Kr,1); % Size of input for later use.
clear data

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initial Conditions
Cqi_0 = eye(3); % DCM of Frame q relative to Frame i
Cbi_0 = eye(3); % DCM of Frame b relative to Frame i
Chi_0 = eye(3); % DCM of Frame h relative to Frame i
r_qi_q_0 = x_d(1:3,1); % Quadcopter position
q_qi_q_0 = Cqi_0(:); % Quadcopter attitude parameters. See above.
q_bi_b_0 = Cbi_0(:); % IP attitude parameters. See above.
q_hi_h_0 = Chi_0(:); % HP attitude parameters. See above.
q_e_0 = zeros(size(const.K_e,1),1); % Temporal flexible states
v_qi_q_0 = x_d(4:6,1); % Quadcopter velocity
w_qi_q_0 = [0 0 0]'; % Quadcopter angular rate
hat_w_bi_b_0 = [0 0]'; % Unconstrained IP angular rate states
hat_w_hi_h_0 = [0 0]'; % Unconstrained HP angular rate states
dot_q_e_0 = zeros(size(const.K_e,1),1); % Rate temporal flexible states
u_true_0 = const.bar_u; % Initial control inputs
NL_IC = [r_qi_q_0; q_qi_q_0; q_bi_b_0; q_hi_h_0; q_e_0;
    v_qi_q_0; w_qi_q_0; hat_w_bi_b_0; hat_w_hi_h_0; dot_q_e_0; u_true_0]; % Initial conditions

u_cmd = zeros(size(Kr,1),t_div_CL); const.u = u_cmd; % Initialize input variable
x_out = zeros(t_div_sim,size(NL_IC,1)); x_out(1,:) = NL_IC; % Initialize output variable

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation
LV1 = 1; % Initialize loop variable 1 for inner-loop con'trol
LV2 = 1; % Initialize loop variable 2 for numerical integration output data storage
options = odeset('AbsTol',1e-11,'RelTol',1e-10); % Set integration tolerences
disp('Starting simulation...')
tic
sim_divs = t_div_CL; % Set number of time steps the inner-loop runs during the simulation
% Run simulation until simulation end time
while LV1 <= sim_divs
    if LV1 < sim_divs
        t_span = t(LV2:(LV2+Nsteps)); % Simulation time for time-step LV1
    else
        t_span = t(LV2:(LV2+Nsteps-1)); % Simulation time for final time step
    end
    
    % Calculate desired control effort
    Cqi = reshape(x_out(LV2,4:12),[3,3]); % Quadcopter DCM
    Cbi = reshape(x_out(LV2,13:21),[3,3]); % IP DCM
    [rq, pq, yq] = DCM2Euler321(Cqi); % [rad] Euler angles of quadcopter used for control
    [rip, pip, ~] = DCM2Euler321(Cbi); % [rad] Euler angles of IP used for control
    x_c = [x_out(LV2,1:3).'; [rq; pq; yq]; [rip; pip]; x_out(LV2,const.size_q+1:const.size_q+size_M_Tw-(4+size(const.K_e,1))).'];
    k = 1 + floor(t(LV2)/dT_traj); % kth time-step of trajectory data
    e = x_d(:,k) - x_c; % State error.
    u_c = Kr*e; % [rpm] Calculate desired control effort from controller. See Eq. (60).
    u_cmd(:,LV1) = u_d(:,k) + u_c; const.u = u_cmd(:,LV1); % [rpm] Total desired control effort
    
    % Numerically integrate over discrete-time time step with continuous-time
    % dynamics
    OL_IC = x_out(LV2,:);
    [t_temp,x_temp] = ode45(@odes,t_span,OL_IC,options,0,const);
    t(LV2:LV2+length(t_temp)-1) = t_temp;
    x_out(LV2:LV2+size(x_temp,1)-1,:) = x_temp;

    LV1 = LV1 + 1; % Propagate discrete-time loop variable forward
    LV2 = LV2 + length(t_span) - 1; % Propagate continuous-time loop variable forward
end

time_stamp = toc

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Save data
save(['.\results\version',num2str(version),'\',config_name,'\sim_data_',config_name,'.mat'])
% version_name = 'RIPplusHP';
% if strcmp(version_name,'RIP')
%     clearvars -except version_name
%     close all
%     clc
% else
%     clearvars -except fig_* version_name
% end
% load(['.\results\',version_name,'\sim_data_',version_name,'.mat']) % Load previously existing data for post-processing. Comment out "Simulation" section
% if ~exist('.\results\co-plot')
%     mkdir('.\results\co-plot')
% end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Post Processing
disp('Starting post processing...')
postprocessing

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot data
disp('Starting plotting...')
% plotscript_alt
plotscript

disp('***Completed***')