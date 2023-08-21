% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set values for discrete-time time-step
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Extract trajectory data
k = 1 + floor(t(LV2)/dT_traj); % kth time-step of trajectory data
rk(:,LV1) = [x_d(1:3,k); x_d(4:6,k); x_d(7:8,k); x_d(9:11,k); x_d(12:14,k); x_d(15:16,k)]; % Desired state values at k
uk = u_d(:,k); % Desired controller inputs at k

% Retrieve states for feedback into estimator
Cqi = reshape(x_out(LV2,4:12),[3,3]); % Quadcopter DCM
Cbi = reshape(x_out(LV2,13:21),[3,3]); % IP DCM
Csi = reshape(x_out(LV2,22:30),[3,3]); % HP DCM
[rq, pq, yq] = DCM2Euler321(Cqi); % [rad] Euler angles of quadcopter used for control
[rip, pip, ~] = DCM2Euler321(Cbi); % [rad] Euler angles of IP used for control
% IP_slope_vec = const.dPSI_0*(x_out(LV2,(31:const.size_q)).'); % Compute the vector resolved in Frame b of the slope at the base of the inverted pendulum 
% rip = rip + atan2(IP_slope_vec(2),IP_slope_vec(3)); % [rad] Add the slope due to flex onto IP rigid body roll measurements
% pip = pip + atan2(IP_slope_vec(1),IP_slope_vec(3)); % [rad] Add the slope due to flex onto IP rigid body pitch measurements

% Calculate desired control effort
x_true = [x_out(LV2,1:3).'; [rq; pq; yq]; [rip; pip]; x_out(LV2,size_q+1:size_q+size_M_Tw-(4+size(const.K_ee,1))).'];
% uff_dot_ud = ((C1(y_out(4))*C2(y_out(5))*C3(y_out(6))*one3).')*one3;
% uff_dot_ud = 1;
% u_ff = (sqrt(abs(uk)/kF)/uff_dot_ud).*ones(4,1); % Feedforward control input
u_ff = const.bar_u; % Feedforward control input (Hover = const.bar_u)
e = rk(:,LV1) - x_true; % State error;
du_c = Kr*e; % [rpm] Calculate desired control effort from controller
u_cmd(:,LV1) = u_ff + du_c; const.u = u_cmd(:,LV1); % [rpm] Total desired control effort

% % a priori (predicted) estimates
% checkx = Ad2*hatx(:,LV1) + Bd2*du_c;
% checky = Cd2*checkx;
% checkP = Ad2*hatP(:,:,LV1)*(Ad2.') + Vd;
% Kf = checkP*(Cd2.')/(Cd2*checkP*(Cd2.') + Vn);
% 
% % a posteriori (corrected) estimates
% y_meas(:,LV1+1) = [x_out(LV2,1:3).'; [rq; pq; yq]; [rip; pip]; x_out(LV2,size_q+1:size_q+8).'] + Vn*randn([size(Vn,1),1]); % Output from sensors corrupted by measruement noise
% hatx(:,LV1+1) = checkx + Kf*(y_meas(:,LV1) - checky); % Propagate controller states forward in time
% hatx_c(:,LV1+1) = hatx(1:end-(size(Ad2,1)-size(Ad1,1)),LV1+1);
% hatP(:,:,LV1+1) = (eye(size(Kf,1)) - Kf*Cd2)*checkP;

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Numerically integrate over discrete-time time-step with continuous-time
% dynamics
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
OL_IC = x_out(LV2,:);
[t_temp,x_temp] = ode45(@odes_DT,t_span,OL_IC,options,0,const);
t(LV2:LV2+length(t_temp)-1) = t_temp;
x_out(LV2:LV2+size(x_temp,1)-1,:) = x_temp;