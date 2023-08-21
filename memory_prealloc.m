% Title: memory_prealloc.m
% Author(s): William Elke III
% Date: 08-Mar-2021
% Description: Script with the purpose of creating variables of zeros to
% preallocate memory for the large variables.
%
% Updated 11-Mar-2021
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Extract constants from const

% Quadcopter
J_Qa = const.J_Qa;
r_d1a_X = const.r_d1a_X;
r_d2a_X = const.r_d2a_X;
r_d3a_X = const.r_d3a_X;
r_d4a_X = const.r_d4a_X;
r_ma_X = const.r_ma_X;
r_ra_X = const.r_ra_X;
kF = const.kF;
kM = const.kM;

% Balanced Pendulum
m_B = const.m_B;
r_em_X = const.r_em_X;
c_Bm_X = const.c_Bm_X;
J_Bm = const.J_Bm;
P_Bm = const.P_Bm;
H_Bm = const.H_Bm;
M_EE = const.M_EE;
K_E = const.K_E;
D_E = const.D_E;

% Hanging Pendulum
m_S = const.m_S;
r_pr_X = const.r_pr_X;
c_Sr_X = const.c_Sr_X;
J_Sr = const.J_Sr;

% Other constants
m_T = const.m_T;
g = const.g;
one3 = const.one3;
size_q = const.size_q; % Size of q calculated from columns of K_E
size_x = const.size_x; % Size of physical states

%% Preallocate memory

Cqi_0 = reshape(NL_IC(4:12),[3,3]); % Quadcopter DCM
Cbi_0 = reshape(NL_IC(13:21),[3,3]); % IP DCM
Chi_0 = reshape(NL_IC(22:30),[3,3]); % HP DCM

% Constraint matrix
P42 = ((one3')*Cqi_0*(Cbi_0')*one3)\(one3'); % Dummy used for size
P43 = ((one3')*Cqi_0*(Cbi_0')*one3)\((one3')*Cqi_0*(Cbi_0')*[1 0; 0 1; 0 0]);
P52 = (one3')*Chi_0*(Cqi_0');

const.Pi = [eye(3) zeros(3,7+length(q_e_0));
    zeros(3) eye(3) zeros(3,4+length(q_e_0));
    zeros(2,6) eye(2) zeros(2,2+length(q_e_0));
    zeros(1,3) P42 P43 zeros(1,2+length(q_e_0));
    zeros(2,8) eye(2) zeros(2,length(q_e_0));
    zeros(1,3) P52 zeros(1,4+length(q_e_0));
    zeros(length(q_e_0),10) eye(length(q_e_0))];

const.dot_Pi = zeros(size(const.Pi));

clear P42 P43 P52 Cqi_0 Cbi_0 Chi_0

% Constrained matrices
const.hat_M = zeros(size((const.Pi')*const.M_Tw*const.Pi));
const.hat_K = zeros(size((const.Pi')*const.K_E));
const.hat_D = zeros(size((const.Pi')*const.D_E*const.Pi));
const.hat_f_non = zeros(size(const.hat_K,1),1);
const.hat_f_C = zeros(size(size(const.hat_K,1),1));

% Other variables

