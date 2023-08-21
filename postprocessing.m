% Title: post_processing.m
% Author(s): William Elke III
% Date: 12-Jan-2021
% Description: Script with the purpose of processing the results of the
% simulation for interpretation via plots
%
% Updated 14-Apr-2022
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Initialize constants
E = zeros(1,length(t));
m_Q = const.m_Q; % Quadcopter mass
r_mq_q = const.r_mq_q; %
r_nq_q = const.r_nq_q;
m_B = const.m_B;
r_bm_b = const.r_bm_b;
K_e = const.K_e;
K_E = const.K_E;
D_E = const.D_E;
P_e = const.P_e;
m_H = const.m_H;
r_hn_h = const.r_hn_h;
g = const.g; % [m/(s^2)] Gravitational constant
one3 = const.one3;
size_q = const.size_q; % Number of rows of q calculated from columns of K_E
size_x = const.size_x; % Number of rows of physical states
e = zeros(size(x_d,1),length(t));

%% Loop over time of simulation
for LV1 = 1:sim_divs
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Set values for discrete-time time-step
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Calculate desired control effort
    const.u = u_cmd(:,LV1); % [rad/s] Total desired control effort

    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Numerically integrate over discrete-time time-step with continuous-time
    % dynamics
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    X = Nsteps*(LV1-1);
    for LV2 = X+1:X+Nsteps
        % Retrieve outputs
        y = odes(t(LV2),x_out(LV2,:)',1,const);
        hat_M = y.hat_M;
        w_bi_b(:,LV2) = y.w_bi_b;
        w_hi_h(:,LV2) = y.w_hi_h;
        u(:,LV2) = y.u;
        u_true(:,LV2) = y.u_true;

        % Retrieve states at each time step for calculations and plotting
        r_qi_i(:,LV2) = (x_out(LV2,1:3)');
        q_qi_q = x_out(LV2,4:12);
        q_bi_b = x_out(LV2,13:21);
        q_hi_h = x_out(LV2,22:30);
        q_e(:,LV2) = x_out(LV2,31:size_q)';
        v_qi_i(:,LV2) = (x_out(LV2,size_q+1:size_q+3)');
        w_qi_q(:,LV2) = x_out(LV2,size_q+4:size_q+6);
        hat_w_bi_b(:,LV2) = x_out(LV2,size_q+7:size_q+8);
        hat_w_hi_h(:,LV2) = x_out(LV2,size_q+9:size_q+10);
        dot_q_e(:,LV2) = x_out(LV2,size_q+11:size_x)';

        hat_nu = [v_qi_i(:,LV2); w_qi_q(:,LV2); hat_w_bi_b(:,LV2); hat_w_hi_h(:,LV2); dot_q_e(:,LV2)];

        % Reshape DCMs
        Cqi = reshape(q_qi_q,[3,3]);
        Cbi = reshape(q_bi_b,[3,3]);
        Chi = reshape(q_hi_h,[3,3]);

        % Compute total energy at each time step
        T_Tw = (1/2).*(hat_nu')*hat_M*hat_nu; % [J] Total kinetic energy

        V_Qw = -m_Q*(g')*(r_qi_i(:,LV2)); % [J] Potential energy of quadcopter

        IP_pin = ((Cqi')*r_mq_q); % [m] Position of the IP pin point relative to the quadcopter CM resolved in inertial frame
        IP_pos = ((Cbi')*r_bm_b); % [m] Position of the IP rigid body CM relative to the IP pin point resolved in inertial frame
        def_U = ((P_e*q_e(:,LV2))'*Cbi*g); % [J] Gravitational potential energy from the deflection of the flexible body CM from the rigid body CM
        V_Bw = (-m_B*(g')*(r_qi_i(:,LV2) + IP_pin + IP_pos)) - def_U + ((1/2).*(q_e(:,LV2)')*K_e*q_e(:,LV2)); % [J] Potential energy of inverted pendulum

        HP_pin = ((Cqi')*r_nq_q); % [m] Position of the HP pin point relative to the quadcopter CM resolved in inertial frame
        HP_pos = ((Chi')*r_hn_h); % [m] Position of the HP CM relative to the HP pin point resolved in inertial frame
        V_Sw = (-m_H*(g')*(r_qi_i(:,LV2) + HP_pin + HP_pos)); % [J] Potential energy of slosh pendulum

        E(LV2) = T_Tw + V_Qw + V_Bw + V_Sw; % [J] Total energy of the system

        % Retrieve Euler Angles from DCMs and check DCM identity
        [quad_roll, quad_pitch, quad_yaw] = DCM2Euler321(Cqi);
        quad_phi(LV2) = quad_roll.*rad2deg;
        quad_theta(LV2) = quad_pitch.*rad2deg;
        quad_psi(LV2) = quad_yaw.*rad2deg;
        quad_DCM_check(LV2) = det(Cqi) - 1;

        Cbq = Cbi*(Cqi');
        [IP_roll, IP_pitch, IP_yaw] = DCM2Euler321(Cbq);
        IP_phi(LV2) = IP_roll.*rad2deg;
        IP_theta(LV2) = IP_pitch.*rad2deg;
        IP_psi(LV2) = IP_yaw.*rad2deg;
        r_bm_q = Cbq'*const.r_bm_b;
        alpha_ip(LV2) = atan2d(r_bm_q(1),-r_bm_q(3));
        beta_ip(LV2) = atan2d(r_bm_q(2),-r_bm_q(3));
        w_bq_b(:,LV2) = w_bi_b(:,LV2) - Cbq*w_qi_q(:,LV2);
        IP_DCM_check(LV2) = det(Cbi) - 1;
        IP_constraint_check(LV2) = (one3')*(w_qi_q(:,LV2) - (Cbq')*w_bi_b(:,LV2));
        IP_ang_vert(LV2) = atan2d(norm(r_bm_q(1:2)),-r_bm_q(3));

        Chq = Chi*(Cqi');
        [HP_roll, HP_pitch, HP_yaw] = DCM2Euler321(Chq);
        HP_phi(LV2) = HP_roll.*rad2deg;
        HP_theta(LV2) = HP_pitch.*rad2deg;
        HP_psi(LV2) = HP_yaw.*rad2deg;
        r_hn_q = Chq'*r_hn_h;
        alpha_hp(LV2) = atan2d(r_hn_q(1),r_hn_q(3));
        beta_hp(LV2) = atan2d(r_hn_q(2),r_hn_q(3));
        w_hq_h(:,LV2) = w_hi_h(:,LV2) - Chq*w_qi_q(:,LV2);
        HP_DCM_check(LV2) = det(Chi) - 1;
        HP_constraint_check(LV2) = (one3')*(w_hi_h(:,LV2) - Chq*w_qi_q(:,LV2));
        HP_ang_vert(LV2) = atan2d(norm(r_hn_q(1:2)),r_hn_q(3));

        % Compute state errors
        [rq, pq, yq] = DCM2Euler321(Cqi); % [rad] Euler angles of quadcopter used for control
        [rip, pip, ~] = DCM2Euler321(Cbi); % [rad] Euler angles of IP used for control
        k = 1 + floor(t(LV2)/dT_traj); % kth time-step of trajectory data
        e(:,LV2) = x_d(:,k) - [x_out(LV2,1:3).'; [rq; pq; yq]; [rip; pip]; x_out(LV2,size_q+1:size_q+size_M_Tw-(4+size(const.K_e,1))).'];
    end
end

w_qi_q = w_qi_q.*rad2deg;
w_bi_b = w_bi_b.*rad2deg;
w_bq_b = w_bq_b.*rad2deg;
w_hi_h = w_hi_h.*rad2deg;
w_hq_h = w_hq_h.*rad2deg;
hat_w_bi_b = hat_w_bi_b.*rad2deg;
hat_w_hi_h = hat_w_hi_h.*rad2deg;
tip_def = (const.PSI_t*q_e).*m2mm; % [mm] Compute tip deflection of inverted pendulum
tip_rate = (const.PSI_t*dot_q_e).*m2mm; % [mm/s] Compute rate of tip deflection of inverted pendulum

