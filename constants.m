% Title: constants_v1.m
% Author(s): William Elke III
% Date: 12-Jan-2021
% Description: Script with the purpose of initializing the constants used
% for the CRQS Testbed in in compile_vX.m
%
% Updated 11-Mar-2021
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Conversions
ft2m = 0.3048; % [(m)/(ft)] Convert from feet to meters
m2ft = 1/ft2m; % [(ft)/(m)] Convert from meters to feet
rad2deg = (180)/(pi); % [(deg)/(rad)] Convert from radians to degrees
deg2rad = 1/rad2deg; % [(rad)/(deg)] Convert from degrees to radians
m2mm = (1e3)/(1); % [(mm)/(m)] Convert from meters to millimeters
lbf2N = (4.44822)/(1); % [(N)/(lbf)] Convert from lbf to N
radps2rpm = ((60)/(1))*((1)/(2*pi)); % [(rpm)/(rad/s)] Convert from rad/s to rpm

%% Load Inverted Pendulum data

if version == 1
    if contains(config_name,'RIP')
        data = load([maindir,'files_for_dynamics\IP_PolyCarb_rigid_1.mat']); % case 1
    elseif contains(config_name,'FIP')
        data = load([maindir,'files_for_dynamics\IP_PolyCarb_flex_1.mat']); % case 1
    else
        disp('Config. error.')
    end
elseif version == 2
    if contains(config_name,'RIP')
        data = load([maindir,'files_for_dynamics\IP_PolyCarb_rigid_2.mat']); % case 2
    elseif contains(config_name,'FIP')
        data = load([maindir,'files_for_dynamics\IP_PolyCarb_flex_2.mat']); % case 2
    else
        disp('Config. error.')
    end
else
    disp('Version error.')
end

const = data.const;
clear data

%% Quadcopter Parameters
% Mass Parameters
const.m_Q = 1.4; % [kg] Mass of quadcopter
Ixx = 0.0181; % [kg*(m^2)] Principal moment of inertia along the q1 axis (roll)
Iyy = 0.0181; % [kg*(m^2)] Principal moment of inertia along the q2 axis (pitch)
Izz = 0.0234; % [kg*(m^2)] Principal moment of inertia along the q3 axis (yaw)
const.J_Qq_q = diag([Ixx,Iyy,Izz]);

% Quadcopter Body Parameters
L = 0.1651; % [m] Length from Cg to motor in q1 and q2 axis. Note: This was given in JSBSim; assumes arms are 45deg intervals and all equivalent length. Trig was already completed.
const.L = L;
h = 0; % [m] Length from Cg to motor in q3 axis
r_d1q_q = [L L -h]'; % [m m m]' Position vector from the quad CM to motor 1 resolved in the Frame q
const.r_d1q_q_X = CrossOp(r_d1q_q);
r_d2q_q = [-L -L -h]'; % [m m m]' Position vector from the quad CM to motor 2 resolved in the Frame q
const.r_d2q_q_X = CrossOp(r_d2q_q);
r_d3q_q = [L -L -h]'; % [m m m]' Position vector from the quad CM to motor 3 resolved in the Frame q
const.r_d3q_q_X = CrossOp(r_d3q_q);
r_d4q_q = [-L L -h]'; % [m m m]' Position vector from the quad CM to motor 4 resolved in the Frame q
const.r_d4q_q_X = CrossOp(r_d4q_q);

D_prop = (9/12)*ft2m; % [m] Propeller diameter
rho = 1.225; % [kg/(m^3)] Air density at STP
CT = 0.1016; % [non-dimensional] Thrust coefficient as a function of J obtained from a table provided for the propeller (see Eq. 6.29a of [B. W. McCormick, Aerodynamics, Aeronautics, and Flight Mechanics, John Wiley & Sons, Inc., 1995.])
CP = 0.0491; % [non-dimensional] Power coefficient as a function of J obtained from a table provided for the propeller (see Eq. 6.29b of [B. W. McCormick, Aerodynamics, Aeronautics, and Flight Mechanics, John Wiley & Sons, Inc., 1995.])
CQ = CP/(2*pi); % [non-dimensional] Moment coefficient (see the note between Eqs. 6.32a and 6.32b of [B. W. McCormick, Aerodynamics, Aeronautics, and Flight Mechanics, John Wiley & Sons, Inc., 1995.])
const.kF = (CT*rho*(D_prop^4))/((2*pi)^2); % [N/((rad/s)^2)] Thrust coefficient of the propellers
const.kM = (CQ*rho*(D_prop^5))/((2*pi)^2); % [(N*m)/((rad/s)^2)] Torque coefficient of the propellers

% Engine Dynamics. To turn off rotor dynamics, modify the corresponding
% code in 'odes.m'.
const.tau_motor = 20e-3; % [s] Time constant of motors
const.motor_LL = sqrt((0.1*lbf2N)/const.kF); % [rad/s] Lower limit of motor speed based on measured static thrust of one rotor
const.motor_UL = sqrt((2.5*lbf2N)/const.kF); % [rad/s] Upper limit of motor speed based on measured static thrust of one rotor

%% Inverted/Balanced Pendulum Parameters
const.r_mq_q = [0 0 -0.03]'; % [m m m]' Position vector from the quad CM to the pin location of the IP resolved in the q frame
const.r_mq_q_X = CrossOp(const.r_mq_q);

const.K_E = [zeros(12,30+size(const.K_e,2));
    zeros(size(const.K_e,1),30) const.K_e];
const.D_E = [zeros(12,12+size(const.D_e,2));
    zeros(size(const.D_e,1),12) const.D_e];

%% Hanging/Slosh Pendulum Parameters

if version == 1
    if contains(config_name,'HP')
        % HP case 1
        const.r_nq_q = [0 0 0.1]'; % [m m m]' Position vector from the quad CM to the pin location of the HP resolved in the q frame
        const.m_H = 0.1; % [kg] Mass of HP
        const.r_hn_h = [0 0 0.2]'; % [m m m]' Position vector from the pin location of the HP to the CM of the HP resolved in the s frame. Length = norm(const.r_hn_h)
    else
        % Turn HP off
        const.r_nq_q = [0 0 0]'; % [m m m]' Position vector from the quad CM to the pin location of the HP resolved in the q frame
        const.m_H = 1e-7; % [kg] Turn HP off
        const.r_hn_h = [0 0 1e7]'; % [m m m]' Turn HP off
    end
elseif version == 2
    if contains(config_name,'HP')
        % HP case 2
        const.r_nq_q = [0 0 0.1]'; % [m m m]' Position vector from the quad CM to the pin location of the HP resolved in the q frame
        const.m_H = 0.1; % [kg] Mass of HP
        const.r_hn_h = [0 0 0.3]'; % [m m m]' Position vector from the pin location of the HP to the CM of the HP resolved in the s frame. Length = norm(const.r_hn_h)
    else
        % Turn HP off
        const.r_nq_q = [0 0 0]'; % [m m m]' Position vector from the quad CM to the pin location of the HP resolved in the q frame
        const.m_H = 1e-7; % [kg] Turn HP off
        const.r_hn_h = [0 0 1e7]'; % [m m m]' Turn HP off
    end
else
    disp('Version error.')
end

const.r_nq_q_X = CrossOp(const.r_nq_q);
const.r_hn_h_X = CrossOp(const.r_hn_h);
const.c_Hn_h_X = const.m_H*const.r_hn_h_X; % [kg*m] First moment of mass of the HP about the pinned location r with the cross operator applied
const.J_Hn_h = -const.m_H*const.r_hn_h_X*const.r_hn_h_X; % [kg*(m^2)] Second moment of mass of the HP about the pinned location r

%% General Parameters
const.m_T = const.m_Q + const.m_B + const.m_H; % [kg] Total mass of the testbed
const.g = [0 0 9.81]'; % [m/(s^2)] Gravitational constant
g_X = CrossOp(const.g);
const.one3 = [0 0 1]';
const.one3_X = CrossOp(const.one3);

%% Prepare Terms for State-Space Form
% Mass matrix
M11 = const.m_T*eye(3);
M12 = -((const.m_B*const.r_mq_q_X) + (const.m_H*const.r_nq_q_X));
M13 = -const.c_Bm_b_X;
M14 = -const.c_Hn_h_X;
M15 = const.P_e;

M21 = M12';
M22 = const.J_Qq_q - (const.m_B*const.r_mq_q_X*const.r_mq_q_X) - (const.m_H*const.r_nq_q_X*const.r_nq_q_X);
M23 = -const.r_mq_q_X*const.c_Bm_b_X;
M24 = -const.r_nq_q_X*const.c_Hn_h_X;
M25 = const.r_mq_q_X*const.P_e;

M31 = M13';
M32 = M23';
M33 = const.J_Bm_b;
M34 = zeros(size(M33,1),size(M13,2));
M35 = const.H_e;

M41 = M14';
M42 = M24';
M43 = M34';
M44 = const.J_Hn_h;
M45 = zeros(size(M44,1),size(M15,2));

M51 = M15';
M52 = M25';
M53 = M35';
M54 = M45';
M55 = const.M_e;

M_Tw = [M11 M12 M13 M14 M15;
    M21 M22 M23 M24 M25;
    M31 M32 M33 M34 M35;
    M41 M42 M43 M44 M45;
    M51 M52 M53 M54 M55];

const.M_Tw = M_Tw;

size_M_Tw = size(M_Tw);

% Gravitational forces
f_g = zeros(size(M_Tw,1),1);
f_g(1:3,1) = const.m_T*const.g;

% Input mapping matrix
hat_B = [-const.kF*const.one3, -const.kF*const.one3, -const.kF*const.one3, -const.kF*const.one3;
    (-const.kF*const.r_d1q_q_X*const.one3 - const.kM*const.one3), (-const.kF*const.r_d2q_q_X*const.one3 - const.kM*const.one3), (-const.kF*const.r_d3q_q_X*const.one3 + const.kM*const.one3), (-const.kF*const.r_d4q_q_X*const.one3 + const.kM*const.one3);
    zeros((size(f_g,1)-6),4)];

% Steady-state input
const.bar_u = sqrt(-pinv(hat_B)*f_g); % Control effort at equilibrium. Note: this is rad/s not (rad/s)^2!!

%% Calculate some vector sizes for later use
const.size_q = size(const.K_E,2); % Number of rows of q calculated from columns of K_E
constrained_states = 2; % Number of constrained states at the rate level
const.size_hat_nu = size(const.D_E,2) - constrained_states; % Number of rows of hat_nu calculated from columns of D_E
const.size_x = const.size_q + const.size_hat_nu; % Number of rows of physical states
const.size_q_e = size(const.K_e,2); % Number of flexible coordinates

%% Pre-allocate memory for Pi matrices
P42 = ((const.one3')*const.one3)\(const.one3'); % Dummy used for size
P43 = ((const.one3')*const.one3)\((const.one3')*[1 0; 0 1; 0 0]);
P52 = (const.one3');

const.Pi = [eye(3) zeros(3,7+const.size_q_e);
    zeros(3) eye(3) zeros(3,4+const.size_q_e);
    zeros(2,6) eye(2) zeros(2,2+const.size_q_e);
    zeros(1,3) P42 P43 zeros(1,2+const.size_q_e);
    zeros(2,8) eye(2) zeros(2,const.size_q_e);
    zeros(1,3) P52 zeros(1,4+const.size_q_e);
    zeros(const.size_q_e,10) eye(const.size_q_e)];

const.dot_Pi = zeros(size(const.Pi));