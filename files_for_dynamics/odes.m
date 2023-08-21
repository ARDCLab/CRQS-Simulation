function  [dot_x] = odes_DT(t,x,output_flag,const)
%ODES
% [dot_x] =ODES_DT(t,x,output_flag,const) returns x_dot = f(x,t) by
% specifying the differential equations of the system in first-order form.
% For use with a discrete-time controller and noise.
%
% INPUT PARAMETERS:
% t = time
% x = system states
% output_flag = changes output of function
% const = a structure that contains all relevant physical parameters
%
% OUTPUT PARAMETERS:
% dot_x = the first-order differential equation evaluated at x and t
%
% Created by: William James Elke III
% Created on: 1-Feb-2021
% Updated on: 14-Apr-2022
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Call constants
% Extract constants from const

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
tau_motor = const.tau_motor;
motor_LL = const.motor_LL;
motor_UL = const.motor_UL;

% Balanced Pendulum
m_B = const.m_B;
r_bm_b_X = const.r_bm_b_X;
c_Bm_b_X = const.c_Bm_b_X;
J_Bm_b = const.J_Bm_b;
P_e = const.P_e;
H_e = const.H_e;
K_E = const.K_E;
D_E = const.D_E;

% Hanging Pendulum
m_H = const.m_H;
r_hn_h_X = const.r_hn_h_X;
c_Hn_h_X = const.c_Hn_h_X;
J_Hn_h = const.J_Hn_h;

% Control
u = const.u;

% Other constants
m_T = const.m_T;
g = const.g;
one3 = const.one3;
size_q = const.size_q; % Size of q calculated from columns of K_E
size_x = const.size_x; % Size of physical states
size_u = const.size_u; % Size of controller states

% Preallocated memory
Pi = const.Pi;
dot_Pi = const.dot_Pi;
M_Tw = const.M_Tw;

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Prepare variables from state vector for use in dynamics

% Extract components of q from x
% r_qi_i(:,1) = x(1:3); % Quadcopter position resolved in inertial frame
q_qi_q(:,1) = x(4:12); % Quadcopter attitude parameters
q_bi_b(:,1) = x(13:21); % IP attitude parameters
q_hi_h(:,1) = x(22:30); % HP attitude parameters
q_e(:,1) = x(31:size_q); % Temporal flexible coordinates

length_q_e = length(q_e);

q(:,1) = x(1:size_q);

% Form DCMs from attitude parameters
Cqi = reshape(q_qi_q,[3,3]); % Quadcopter DCM
Cbi = reshape(q_bi_b,[3,3]); % IP DCM
Chi = reshape(q_hi_h,[3,3]); % HP DCM

% Extract components of hat_nu from x
v_qi_i(:,1) = x(size_q+1:size_q+3); % Quadcopter velocity resolved in inertial frame
w_qi_q(:,1) = x(size_q+4:size_q+6); % Quadcopter angular velocity
hat_w_bi_b(:,1) = x(size_q+7:size_q+8); % Unconstrained IP angular velocity states
hat_w_hi_h(:,1) = x(size_q+9:size_q+10); % Unconstrained HP angular velocity states
dot_q_e(:,1) = x(size_q+11:size_x); % Time rate of change of the temporal flexible coordinates

hat_nu = [v_qi_i; w_qi_q; hat_w_bi_b; hat_w_hi_h; dot_q_e]; % nu with all unconstrained states

% Form constraint matrix, Pi
Pi(9,4:6) = ((one3')*Cqi*(Cbi')*one3)\(one3');
Pi(9,7:8) = -((one3')*Cqi*(Cbi')*one3)\((one3')*Cqi*(Cbi')*[1 0; 0 1; 0 0]);
Pi(12,4:6) = (one3')*Chi*(Cqi');

% Reconstruct nu from Pi and hat_nu
nu = Pi*hat_nu; % nu including all unconstrained and constrained states

% Extract all three components of the angular velocities of the IP and HP
% from the reconstructed nu which include constrained state(s)
w_bi_b(:,1) = nu(7:9);
w_hi_h(:,1) = nu(10:12);

% Apply cross operator to be used for cross product operations
w_qi_q_X = CrossOp(w_qi_q);
w_bi_b_X = CrossOp(w_bi_b);
w_hi_h_X = CrossOp(w_hi_h);

% Form time rate of change of the constraint matrix, dot_Pi
dot_Pi(9,4:6) = -(((one3')*Cqi*(Cbi')*one3)^-2)*(one3')*((-w_qi_q_X*Cqi*(Cbi'))+(Cqi*(Cbi')*w_bi_b_X))*one3*(one3');
dot_Pi(9,7:8) = (-(((one3')*Cqi*(Cbi')*one3)^-2)*(one3')*((-w_qi_q_X*Cqi*(Cbi'))+(Cqi*(Cbi')*w_bi_b_X))*one3*(one3')*Cqi*(Cbi')*[1 0; 0 1; 0 0]) - ((((one3')*Cqi*(Cbi')*one3)^-1)*(one3')*((-w_qi_q_X*Cqi*(Cbi'))+(Cqi*(Cbi')*w_bi_b_X))*[1 0; 0 1; 0 0]);
dot_Pi(12,4:6) = (one3')*((-w_hi_h_X*Chi*(Cqi'))+(Chi*(Cqi')*w_qi_q_X));

% Use Poisson's equation to compute dot_DCMs
dot_Cqi = -w_qi_q_X*Cqi;
dot_Cbi = -w_bi_b_X*Cbi;
dot_Chi = -w_hi_h_X*Chi;

% Form dot_q to be used to propagate state vector
dot_q = [v_qi_i;
    dot_Cqi(:);
    dot_Cbi(:);
    dot_Chi(:);
    dot_q_e];

% Form mass matrix
M_Tw(1:3,4:6) = -(Cqi')*((m_B*r_mq_q_X) + (m_H*r_nq_q_X));
M_Tw(1:3,7:9) = -(Cbi')*c_Bm_b_X;
M_Tw(1:3,10:12) = -(Chi')*c_Hn_h_X;
M_Tw(1:3,12+1:12+length_q_e) = (Cbi')*P_e;

M_Tw(4:6,1:3) = M_Tw(1:3,4:6)';
M_Tw(4:6,7:9) = -r_mq_q_X*Cqi*(Cbi')*c_Bm_b_X;
M_Tw(4:6,10:12) = -r_nq_q_X*Cqi*(Chi')*c_Hn_h_X;
M_Tw(4:6,12+1:12+length_q_e) = r_mq_q_X*Cqi*(Cbi')*P_e;

M_Tw(7:9,1:3) = M_Tw(1:3,7:9)';
M_Tw(7:9,4:6) = M_Tw(4:6,7:9)';

M_Tw(10:12,1:3) = M_Tw(1:3,10:12)';
M_Tw(10:12,4:6) = M_Tw(4:6,10:12)';

M_Tw(12+1:12+length_q_e,1:3) = M_Tw(1:3,12+1:12+length_q_e)';
M_Tw(12+1:12+length_q_e,4:6) = M_Tw(4:6,12+1:12+length_q_e)';

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Dynamics
% Compute nonlinear forces/torques
f_non = [(m_T*g) + ((Cqi')*w_qi_q_X*((m_B*r_mq_q_X)+(m_H*r_nq_q_X))*w_qi_q) + ((Cbi')*w_bi_b_X*c_Bm_b_X*w_bi_b) + ((Chi')*w_hi_h_X*c_Hn_h_X*w_hi_h) + (-(Cbi')*w_bi_b_X*P_e*dot_q_e);
    (m_H*r_nq_q_X*Cqi*g) + (m_B*r_mq_q_X*Cqi*g) + (r_mq_q_X*Cqi*(Cbi')*w_bi_b_X*c_Bm_b_X*w_bi_b) + (r_nq_q_X*Cqi*(Chi')*w_hi_h_X*c_Hn_h_X*w_hi_h) + (-r_mq_q_X*Cqi*(Cbi')*w_bi_b_X*P_e*dot_q_e) + (-w_qi_q_X*(J_Qq_q - (m_B*r_mq_q_X*r_mq_q_X) - (m_H*r_nq_q_X*r_nq_q_X))*w_qi_q);
    (m_B.*r_bm_b_X*Cbi*g) + (CrossOp(P_e*q_e)*Cbi*g) + (c_Bm_b_X*Cbi*(Cqi')*w_qi_q_X*r_mq_q_X*w_qi_q) + (-w_bi_b_X*J_Bm_b*w_bi_b) + (-w_bi_b_X*H_e*dot_q_e) + (-CrossOp(Cbi*v_qi_i)*P_e*dot_q_e) + (-CrossOp(P_e*dot_q_e)*Cbi*(Cqi')*r_mq_q_X*w_qi_q);
    (m_H*r_hn_h_X*Chi*g) + (c_Hn_h_X*Chi*(Cqi')*w_qi_q_X*r_nq_q_X*w_qi_q) + (-w_hi_h_X*J_Hn_h*w_hi_h);
    ((P_e')*Cbi*g) + ((P_e')*w_bi_b_X*Cbi*v_qi_i) + (-(P_e')*w_bi_b_X*Cbi*(Cqi')*r_mq_q_X*w_qi_q) + ((P_e')*Cbi*(Cqi')*w_qi_q_X*r_mq_q_X*w_qi_q)];

% Apply first-order motor dynamics
u_true = x(size_x+1:size_x+size_u,1); % [rad/s] Extract true control effort
dot_u_true = (1/tau_motor).*(u - u_true); % [rad/(s^2)] Apply first-order filter to replicate motor dynamics
% u_true = u(:,1); % Uncomment to turn motor dynamics off
% dot_u_true = zeros(size(u_true)); % Uncomment to turn motor dynamics off

% Saturate motors
u_true(u_true < motor_LL) = motor_LL; % [rad/s] Saturate motor output. Comment out to eliminate saturation.
u_true(u_true > motor_UL) = motor_UL; % [rad/s] Saturate motor output. Comment out to eliminate saturation.

% Compute mapping matrix for control forces/torques
hat_B = [-kF*(Cqi')*one3, -kF*(Cqi')*one3, -kF*(Cqi')*one3, -kF*(Cqi')*one3;
    (-kF*r_d1q_q_X*one3 - kM*one3), (-kF*r_d2q_q_X*one3 - kM*one3), (-kF*r_d3q_q_X*one3 + kM*one3), (-kF*r_d4q_q_X*one3 + kM*one3);
    zeros((size(f_non,1)-6),4)]; % Mapping matrix that maps control inputs (i.e. squared propeller speeds) to control forces/torques

% Calculate control forces/torques
f_C = hat_B*(u_true.^2); % Control forces/torques

% Enforce constraints
hat_M = (Pi')*M_Tw*Pi;
hat_K = (Pi')*K_E;
hat_D = (Pi')*D_E*Pi;
hat_f_non = (Pi')*f_non + (Pi')*M_Tw*dot_Pi*hat_nu;
hat_f_C = (Pi')*f_C;

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Form dot_x = f(x,u) system.
dot_hat_nu = hat_M\(hat_f_C + hat_f_non - (hat_D*hat_nu) - (hat_K*q));

x_dot = [dot_q;
    dot_hat_nu;
    dot_u_true]; % Motor states

%% Select output
if (output_flag == 0) % Use this for numerical integration
    dot_x = x_dot;
    
else % Use this for post-processing
    y.hat_M = hat_M;
    y.w_bi_b = w_bi_b;
    y.w_hi_h = w_hi_h;
    y.u = u;
    y.u_true = u_true;
    
    dot_x = y;
end
end



