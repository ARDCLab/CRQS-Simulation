% Title: ControlParamCalc.m
% Author(s): William Elke III
% Date: 18-Feb-2021
% Description: Script with the purpose of calculating the controller
% parameters
%
% Updated 11-Mar-2021
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clear
clc

% Add paths for necessary scripts
maindir = [pwd,'\']; % Make sure you are in the main directory first! NOT the control directory!
addpath([maindir,'files_for_control'],...
            [maindir,'files_for_dynamics'],[maindir,'files_for_support'],...
            [maindir,'files_for_trajectory'],[maindir,'results'])

%% Define Constants and Initialize Variables
version = 2;
config_name = 'RIP'; % Do not change this
constants; % Run constants script

% Extract step size from trajectory data
data = load([maindir,'files_for_trajectory\trajectory_data.mat']);
dT = data.t_traj(2) - data.t_traj(1); % [s] Trajectory time step size
clear data

% Extract constants from 'const'
% Quadcopter
J_Qq_q = const.J_Qq_q;
r_d1q_q_X = const.r_d1q_q_X;
r_d2q_q_X = const.r_d2q_q_X;
r_d3q_q_X = const.r_d3q_q_X;
r_d4q_q_X = const.r_d4q_q_X;
r_mq_q_X = const.r_mq_q_X;
r_nq_q_X = const.r_nq_q_X;
kF = const.kF;
kM = const.kM;
motor_LL = const.motor_LL;
motor_UL = const.motor_UL;

% Balanced Pendulum
m_B = const.m_B;
r_bm_b_X = const.r_bm_b_X;
c_Bm_b_X = const.c_Bm_b_X;
J_Bm_b = const.J_Bm_b;
P_e = const.P_e;
H_e = const.H_e;
M_e = const.M_e;
K_e = const.K_e;
D_e = const.D_e;
K_E = [zeros(12,12+size(K_e,2));
    zeros(size(K_e,1),12) K_e];
D_E = [zeros(12,12+size(D_e,2));
    zeros(size(D_e,1),12) D_e];

% Hanging Pendulum
m_H = const.m_H;
r_hn_h_X = const.r_hn_h_X;
c_Hn_h_X = const.c_Hn_h_X;
J_Hn_h = const.J_Hn_h;

% Other constants
m_T = const.m_T;
g = const.g;
g_X = CrossOp(g);
one3 = const.one3;
one3_X = const.one3_X;
bar_u = const.bar_u;

% Partial derivatives
dBdq = zeros(size(K_E,1),size(K_E,2));
dBdq(1:3,4:6) = kF*sum(bar_u.^2)*one3_X; % Note: bar_u has units of rpm not rpm^2! Yes, that is not a mistake!

dfdq = zeros(size(K_E,1),size(K_E,2));
dfdq(4:6,4:6) = (m_H*r_nq_q_X*g_X) + (const.m_B*r_mq_q_X*g_X);
dfdq(7:9,7:9) = const.m_B*const.r_bm_b_X*g_X;
dfdq(10:12,10:12) = m_H*r_hn_h_X*g_X;
dfdq(13:size(K_E,1),7:9) = (const.P_e')*g_X;

dfdnu = zeros(size(D_E,1),size(D_E,2));

%% Enforce Constraints
% IP and HP cannot freely rotate about the quadcopter's 3-axis
% Assume structural dynamics and hanging pendulum are disturbances:
%   - Quadcopter position (x3)
%   - Quadcopter attitude (x3)
%   - Inverted pendulum attitude (x2)
%   - Quadcopter velocity (x3)
%   - Quadcopter angular velocity (x3)
%   - Inverted pendulum angular velocity (x2)
M_Tw = M_Tw(1:8,1:8);
K_E = K_E(1:8,1:8);
D_E = D_E(1:8,1:8);
dBdq = dBdq(1:8,1:8);
dfdq = dfdq(1:8,1:8);
dfdnu = dfdnu(1:8,1:8);
hat_B = hat_B(1:8,1:size(hat_B,2));

%% State-Space matrices
M_Tw_inv = eye(size(M_Tw))/(M_Tw);

A = [zeros(size(K_E,1),size(K_E,2)) eye(size(D_E,1),size(D_E,2));
    M_Tw_inv*(dBdq + dfdq - K_E) M_Tw_inv*(dfdnu - D_E)];

B = [zeros(size(A,1)-size(hat_B,1),size(hat_B,2));
    2.*M_Tw_inv*hat_B*diag(bar_u)];

C = eye(size(A,1),size(A,2));

D = zeros(size(A,1),size(B,2));

G_fs = ss(A,B,C,D);
Gd = c2d(G_fs,dT,'zoh');

% % Check Condition Number
% [U,S,V] = svd(A);
% S_max = S(1,1);
% S_min = S(size(A,1),size(A,2));
% gamma = S_max/S_min
% 
% % Check controllability
% size(ctrb(A,B))
% rank_ctrb = rank(ctrb(A,B))
% 
% % Check observability
% size(obsv(A,C))
% rank_obsv = rank(obsv(A,C))

% Check poles of plant
% poles_G = pole(Gd)
% figure
% hold on
% pzplot(Gd);

% damp(Gd)

%% Linear Quadratic Regulator
% State weighting matrix
Q = zeros(size(A));
Q(1:3,1:3) = 100.*diag([1 1 1]); % r_aw_i
Q(4:6,4:6) = 1.*diag([1 1 1]); % Theta_qi
Q(7:8,7:8) = 1.*diag([1 1]); % Theta_bi
Q(9:11,9:11) = 10.*diag([1 1 1]); % v_aw_i
Q(12:14,12:14) = 1.*diag([1 1 1]); % w_qi
Q(15:16,15:16) = 1.*diag([1 1]); % hat_w_bi

% Input weighting matrix
R = 1e-4*eye(size(B,2));

% Compute gain matrix
Kr = lqr(Gd,Q,R);
K_sys = ss(Kr);

% Evaluate closed-loop system
CL_sys = feedback(Gd*K_sys,eye(size(Gd*K_sys)));
% pole_CL = pole(CL_sys)
% [V,D] = eig(CL_sys.A,CL_sys.B);
% figure
% hold on
% pzplot(CL_sys);

% [wn,~,~] = damp(CL_sys(:,:))

opts = bodeoptions;
opts.FreqUnits = 'Hz';
opts.PhaseVisible = 'off';
h = bodeplot(CL_sys(12,[1 4 7]),opts);

% save([maindir,'files_for_control\control_params_',num2str(version)],'Kr')

disp('***Completed***')


