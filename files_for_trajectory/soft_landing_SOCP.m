% Title: soft_landing_SOCP.m
% Author(s): William Elke III
% Date: 08-Dec-2021
% Description: Script with the purpose of calculating the
% trajectory for the CRQS in compile.m. This script addresses the
% soft-landing trajectory using a Second-Order Cone Program (SOCP).
%
% Second-Order Cone Program Information:
% https://www.mathworks.com/help//optim/ug/coneprog.html
%
% Updated 02-Jul-2023
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% delete(gcp('nocreate'))
close all
clear
clc

% Add paths to necessary scripts
maindir = [pwd,'\']; % Make sure you are in the main directory first! NOT the trajectory directory!
addpath([maindir,'files_for_control'],...
    [maindir,'files_for_dynamics'],[maindir,'files_for_support'],...
    [maindir,'files_for_trajectory'],[maindir,'results'])


version = 2; % Do not change this.
config_name = 'RIP'; % Do not change this.

if ~exist(['.\files_for_trajectory\version',num2str(version)],"dir")
    mkdir(['.\files_for_trajectory\version',num2str(version)])
end

%% Define constants
constants
m = const.m_Q + const.m_B; % [kg] Mass of quad + IP
g = const.g;
one_12 = [1 0;
    0 1;
    0 0];

rd = zeros(2,1); % [m m] Desired landing location
rho_min = (4*const.kF*(1.25*const.motor_LL)^2); % [N] Minimum thrust
rho_max = (4*const.kF*(0.75*const.motor_UL)^2); % [N] Maximum thrust
gamma_gs = 20; % [deg] Glideslope angle
c_gs = [0; 0; -1/tand(gamma_gs)];
Vmax = 2; % [m/s] Maximum quadcopter velocity
theta = 20; % [deg] Thrust pointing constraint of quadcopter
cosd_theta = cosd(theta);
hat_n = [0; 0; -1]; % [-] Nominal thrust pointing direction

x0 = [-3; 0; -3; 0; 0; 0]; % Initial position and velocity

% Simulation time
t0 = 0; % [s] Start time
t_max = 15; % [s] Final time
dT = 0.05; % [s] Time step
N0 = round(t_max/dT); % Maximum flight time that will be iteratively decreased during the line search
N_star = 5/dT; % Desired trajectory run time for a single run without iterations

%% Describe system
% Continuous time dynamics
Ac = [zeros(3) eye(3);
    zeros(3) zeros(3)];
Bc = [zeros(3);
    eye(3)];
Cc = eye(size(Ac,2));
Gc = ss(Ac,Bc,Cc,0);

% Discrete time dynamics
Gd = c2d(Gc,dT,'zoh');
Ad = Gd.A;
nx = size(Ad,1);
Bd = Gd.B;
nu = size(Bd,2); % Dimension of input

y_star_MLE_temp = [];
fval_MLE_temp = [];
y_star_MF_temp = [];
fval_MF_temp = [];
exitflag_MF = 1;
LV3 = 1;
disp('***Beginning Minimum Time of Flight Iterations...***')
tic

% For the following while loop, change the conditional to N > 0 to have the
% loop complete the line search from N0 defined above until the simulation
% cannot find a solution. Change the conditional to N == N_star defined
% above to have the loop complete only one pass at the specified time.

% N = N0; % Uncomment this line and change the conditional to perform the line search.
N = N_star; % Uncomment this line and change the conditioanl to perform one pass.
while N == N_star % [N > 0 (iterate through tf), N == N_star (run only once for desired tf, N_star)]
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Minimum Landing Error Problem
    disp('% %%%%%%%')
    clear socConstraints

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Initialize variables for optimization
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    U0 = zeros(nu*N,1);
    for LV1 = 1:N
        U0 = [U0; rho_min.*[0; 0; -1] ];
    end
    y0 = [0; U0; rho_min.*ones(N,1)]; % [eta0; U0; Gamma_p_0]

    % Setup up simulation
    % Form B matrices for simulation
    B1 = Bd;
    B2 = (1/m).*Bd;

    % Construct matrices for simulation
    S1 = zeros(N*nx,N*size(B1,2));
    S2 = zeros(N*nx,N*size(B2,2));
    M = zeros(N*nx,size(Ad,2));
    W = zeros(3*N,1);
    % disp('***Beginning Construction of Simulation Matrices...***')
    % tic
    for LV1 = 1:N
        % S1
        for LV2 = 1:LV1
            S1(nx*(LV1-1)+1:nx*LV1, nu*(LV2-1)+1:nu*LV2) = Ad^(LV1-LV2)*B1;
        end

        % S2
        for LV2 = 1:LV1
            S2(nx*(LV1-1)+1:nx*LV1, nu*(LV2-1)+1:nu*LV2) = Ad^(LV1-LV2)*B2;
        end

        % M
        M(nx*(LV1-1)+1:nx*LV1,:) = Ad^LV1;

        % W
        W(3*(LV1-1)+1:3*LV1,:) = g;
    end
    % toc

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Set up SOC constraints. Start with the final index for computational
    % purposes.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % disp('***Beginning Formulation of SOC Constraints...***')
    % tic
    socConstraints(1 + N + N-1 + N) = secondordercone([],[],[],0);

    % Cost function
    MrN = [zeros(3,nx*(N-1)) eye(3) zeros(3)];
    Ups_U = [zeros(nu*N,1) eye(nu*N) zeros(nu*N,N)];
    Asc = (one_12.')*MrN*S2*Ups_U;
    bsc = rd - (one_12.')*MrN*((S1*W)+(M*x0));
    dsc = [1 zeros(1,size(Asc,2)-1)].';
    gamma = 0;
    socConstraints(1) = secondordercone(Asc,bsc,dsc,gamma);
    LVsoc = 1;

    % Glideslope constraint
    for LV1 = 1:N
        k = LV1 - 1;
        Mrk = [zeros(3,nx*k) eye(3) zeros(3) zeros(3,nx*(N-k-1))];
        Asc = (one_12.')*(Mrk - MrN)*S2*Ups_U;
        bsc = -(one_12.')*(Mrk - MrN)*((S1*W)+(M*x0));
        dsc = (c_gs'*(Mrk - MrN)*S2*Ups_U)';
        gamma = -(c_gs.')*(Mrk - MrN)*((S1*W)+(M*x0));
        socConstraints(LVsoc + LV1) = secondordercone(Asc,bsc,dsc,gamma);
    end
    LVsoc = LVsoc + N;

    % Maximum velocity constraint
    for LV1 = 1:N-1
        k = LV1 - 1;
        Mdrk = [zeros(3,nx*k) zeros(3) eye(3) zeros(3,nx*(N-k-1))];
        Asc = Mdrk*S2*Ups_U;
        bsc = -Mdrk*((S1*W)+(M*x0));
        dsc = zeros(size(Asc,2),1);
        gamma = -Vmax;
        socConstraints(LVsoc + LV1) = secondordercone(Asc,bsc,dsc,gamma);
    end
    LVsoc = LVsoc + N-1;

    % Thrust magnitude constraint
    for LV1 = 1:N
        k = LV1 - 1;
        Ups_uk = [zeros(nu,1) zeros(nu,nu*k) eye(nu) zeros(nu,nu*(N-k-1)) zeros(nu,N)];
        Ups_gk = [0 zeros(1,nu*N) zeros(1,k) 1 zeros(1,(N-k-1))];
        Asc = Ups_uk;
        bsc = zeros(size(Asc,1),1);
        dsc = Ups_gk.';
        gamma = 0;
        socConstraints(LVsoc + LV1) = secondordercone(Asc,bsc,dsc,gamma);
    end
    % toc

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Set up linear constraints
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % disp('***Beginning Formulation of Linear Constraints...***')
    % tic
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    LB = [];
    UB = [];

    % Landing altitude constraint
    Aeq = [Aeq; [0 0 1]*MrN*S2*Ups_U];
    beq = [beq; -[0 0 1]*MrN*((S1*W)+(M*x0))];

    % Landing velocity constraint
    MdrN = [zeros(3,nx*(N-1)) zeros(3) eye(3)];
    Aeq = [Aeq; MdrN*S2*Ups_U];
    beq = [beq; -MdrN*((S1*W)+(M*x0))];

%     % Don't-go-too-high constraint
%     alt_max = abs(x0(3)) + 1;
%     for LV1 = 1:N-1
%         k = LV1 - 1;
%         Mrk = [zeros(3,nx*k) eye(3) zeros(3) zeros(3,nx*(N-k-1))];
%         A = [A; -[0 0 1]*Mrk*S2*Ups_U];
%         b = [b; alt_max+[0 0 1]*Mrk*((S1*W)+(M*x0))];
%     end

    % Thrust pointing constraint
    Ups_uk = [zeros(nu,1) eye(nu) zeros(nu,nu*(N-1)) zeros(nu,N)];
    Ups_gk = [zeros(1,1) zeros(1,nu*N) 1 zeros(1,(N-1))];
    Aeq = [Aeq; Ups_gk - (hat_n.'*Ups_uk)];
    beq = [beq; 0];
    for LV1 = 2:N
        k = LV1 - 1;
        Ups_uk = [zeros(nu,1) zeros(nu,nu*k) eye(nu) zeros(nu,nu*(N-k-1)) zeros(nu,N)];
        Ups_gk = [zeros(1,1) zeros(1,nu*N) zeros(1,k) 1 zeros(1,(N-k-1))];
        A = [A; (cosd_theta.*Ups_gk) - (hat_n.'*Ups_uk)];
        b = [b; 0];
    end
    Ups_uk = [zeros(nu,1) zeros(nu,nu*(N-1)) eye(nu) zeros(nu,N)];
    Ups_gk = [zeros(1,1) zeros(1,nu*N) zeros(1,(N-1)) 1];
    Aeq = [Aeq; Ups_gk - (hat_n.'*Ups_uk)];
    beq = [beq; 0];

    % Slack variable constraint
    for LV1 = 1:N
        k = LV1 - 1;
        Ups_gk = [zeros(1,1) zeros(1,nu*N) zeros(1,k) 1 zeros(1,(N-k-1))];
        A = [A; [Ups_gk; -Ups_gk]];
        b = [b; [rho_max; -rho_min]];
    end
    % toc

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Solve Second-Order Cone Program
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Linear objective function vector
    f = [1 zeros(1,nu*N) zeros(1,N)]';

    % Solve
    options = [];
%     options = optimoptions('coneprog','ConstraintTolerance',1e-7,'OptimalityTolerance',1e-6);
    % disp('***Solving Minimum Landing Error Problem...***')
    % tic
    [y_star_MLE_temp,fval_MLE_temp,exitflag_MLE] = coneprog(f,socConstraints,A,b,Aeq,beq,LB,UB,options);
    % toc
    disp(['fval_MLE_temp = ',num2str(fval_MLE_temp)])
    disp(['exitflag_MLE = ',num2str(exitflag_MLE)])
    if exitflag_MLE == -2
%         N = N-1;
        break
    end
    delta_star_MLE(LV3) = fval_MLE_temp; % Minimum landing error

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Minimum Fuel Problem

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Adjust constraints from Minimum Landing Error problem
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Minimum landing error constraint
    MrN = [zeros(3,nx*(N-1)) eye(3) zeros(3)];
    Asc = (one_12.')*MrN*S2*Ups_U;
    bsc = rd - (one_12.')*MrN*((S1*W)+(M*x0));
    dsc = zeros(size(Asc,2),1);
    gamma = -delta_star_MLE(LV3);
    socConstraints(1,1) = secondordercone(Asc,bsc,dsc,gamma); % Replace SOC constraint for MLE

%     % Don't-go-too-high constraint
%     alt_max = abs(x0(3)) + 0.5;
%     for LV1 = 1:N-1
%         k = LV1 - 1;
%         Mrk = [zeros(3,nx*k) eye(3) zeros(3) zeros(3,nx*(N-k-1))];
%         A = [A; -[0 0 1]*Mrk*S2*Ups_U];
%         b = [b; alt_max+[0 0 1]*Mrk*((S1*W)+(M*x0))];
%     end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Solve Second-Order Cone Program
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Linear objective function vector
    f = [0 zeros(1,nu*N) ones(1,N)]';

    % Solve
    % disp('***Solving Minimum Fuel Problem...***')
    % tic
    [y_star_MF_temp,fval_MF_temp,exitflag_MF] = coneprog(f,socConstraints,A,b,Aeq,beq,LB,UB,options);
    % toc
    disp(['fval_MF_temp = ',num2str(fval_MF_temp)])
    disp(['exitflag_temp = ',num2str(exitflag_MF)])
    if exitflag_MF == -2
%         N = N-1;
        break
    end
    y_star_MLE = y_star_MLE_temp;
    y_star_MF = y_star_MF_temp;
    fval_MF(LV3) = fval_MF_temp;

    % Iterate time
    disp(['N = ',num2str(N)])
    N = N-1;
    LV3 = LV3 + 1;
end
toc
N = N+1;
U0 = zeros(nu*N,1);
for LV1 = 1:N
    U0 = [U0; rho_min.*[0; 0; -1] ];
end
y0 = [0; U0; rho_min.*ones(N,1)]; % [eta0; U0; Gamma_p_0]

% Setup up simulation
% Form B matrices for simulation
B1 = Bd;
B2 = (1/m).*Bd;

% Construct matrices for simulation
S1 = zeros(N*nx,N*size(B1,2));
S2 = zeros(N*nx,N*size(B2,2));
M = zeros(N*nx,size(Ad,2));
W = zeros(3*N,1);
% disp('***Beginning Construction of Simulation Matrices...***')
% tic
for LV1 = 1:N
    % S1
    for LV2 = 1:LV1
        S1(nx*(LV1-1)+1:nx*LV1, nu*(LV2-1)+1:nu*LV2) = Ad^(LV1-LV2)*B1;
    end

    % S2
    for LV2 = 1:LV1
        S2(nx*(LV1-1)+1:nx*LV1, nu*(LV2-1)+1:nu*LV2) = Ad^(LV1-LV2)*B2;
    end

    % M
    M(nx*(LV1-1)+1:nx*LV1,:) = Ad^LV1;

    % W
    W(3*(LV1-1)+1:3*LV1,:) = g;
end
% toc

%% Post-processing
U_MLE = y_star_MLE(2:(nu*N)+1);
X_MLE = (S1*W) + (S2*U_MLE) + (M*x0);
for LV1 = 1:N
    r_MLE(:,LV1) = X_MLE((LV1-1)*nx+1:(LV1*nx)-3,1);
end
for LV1 = 2:N+1
    socCheck_MLE(LV1) = norm(socConstraints(1,LV1).A*y_star_MLE - socConstraints(1,LV1).b) - (socConstraints(1,LV1).d'*y_star_MLE - socConstraints(1,LV1).gamma);
end
for LV1 = 2:N+1
    socCheck_MF(LV1) = norm(socConstraints(1,LV1).A*y_star_MF - socConstraints(1,LV1).b) - (socConstraints(1,LV1).d'*y_star_MF - socConstraints(1,LV1).gamma);
end
for LV1 = 1:N
    k(LV1) = LV1 - 1;
    dot_r_MLE(:,LV1) = X_MLE((LV1-1)*nx+4:(LV1*nx),1);
    Tc_MLE(:,LV1) = U_MLE((LV1-1)*nu+1:(LV1*nu),1);
    normTc_MLE(LV1) = norm(Tc_MLE(:,LV1));
    theta_hat_MLE(LV1) = (180/pi)*acos(hat_n.'*Tc_MLE(:,LV1)/normTc_MLE(LV1));
    r12_MLE(LV1) = norm(one_12'*(r_MLE(:,LV1)-r_MLE(:,end)));
    GS_MLE(LV1) = c_gs.'*r_MLE(:,LV1);
%     GSd_MLE(LV1) = atan(abs(r_MLE(3,LV1)/norm(r_MLE(1:2,LV1)-r_MLE(1:2,end))));
    GSd_MLE(LV1) = atan2d(-(r_MLE(3,LV1)-r_MLE(3,end)),norm(one_12'*(r_MLE(:,LV1)-r_MLE(:,end))));
    normV_MLE(LV1) = norm(dot_r_MLE(:,LV1));
end
k = [k N];
r_MLE = [x0(1:3) r_MLE];
r_MLE(3,:) = -r_MLE(3,:);
dot_r_MLE = [x0(4:6) dot_r_MLE];
dot_r_MLE(3,:) = -dot_r_MLE(3,:);
normV_MLE = [norm(x0(4:6)) normV_MLE];
GS_MLE = [c_gs.'*x0(1:3) GS_MLE];
% GSd_MLE = [atan(abs(x0(3)/norm(x0(1:2)))) GSd_MLE];
GSd_MLE = ([atan2d(-x0(3),norm(x0(1:2))) GSd_MLE]);

U_MF = y_star_MF(2:(nu*N)+1);
X_MF = (S1*W) + (S2*U_MF) + (M*x0);

for LV1 = 1:N
    r_MF(:,LV1) = X_MF((LV1-1)*nx+1:(LV1*nx)-3,1);
end
for LV1 = 1:N
    k(LV1) = LV1 - 1;
    dot_r_MF(:,LV1) = X_MF((LV1-1)*nx+4:(LV1*nx),1);
    Tc_MF(:,LV1) = U_MF((LV1-1)*nu+1:(LV1*nu),1);
    normTc_MF(1,LV1) = norm(Tc_MF(:,LV1));
    nTc = Tc_MF(:,LV1)/normTc_MF(LV1);
    theta_hat_MF(LV1) = (180/pi)*acos(hat_n.'*nTc);
    roll_d(1,LV1) = asin(sqrt((nTc(1)^2) + (nTc(2)^2)));
    pitch_d(1,LV1) = atan(nTc(2)/nTc(1));
    r12_MF(LV1) = norm(r_MF(1:2,LV1)-r_MF(1:2,end));
    GS_MF(LV1) = c_gs.'*r_MF(:,LV1);
    GSd_MF(LV1) = atan2d(-(r_MF(3,LV1)-r_MF(3,end)),norm(one_12'*(r_MF(:,LV1)-r_MF(:,end))));
    normV_MF(LV1) = norm(dot_r_MF(:,LV1));
end
r_MF = [x0(1:3) r_MF];
dot_r_MF = [x0(4:6) dot_r_MF];
x_d = [r_MF; [0 roll_d]; [0 pitch_d]; zeros(1,N+1); zeros(2,N+1); dot_r_MF; zeros(3,N+1); zeros(2,N+1)];
u_d = Tc_MF(3,:);
r_MF(3,:) = -r_MF(3,:);
dot_r_MF(3,:) = -dot_r_MF(3,:);
normV_MF = [norm(x0(4:6)) normV_MF];
r12_MF = [norm(x0(1:2)-r_MF(1:2,end)) r12_MF];
GS_MF = [c_gs.'*x0(1:3) GS_MF];
GSd_MF = ([atan2d(-x0(3),norm(x0(1:2))) GSd_MF]);
delta_star_MF = r12_MF(end)

%% Save data
t_traj = dT.*(0:N);
% save([maindir,'files_for_trajectory\version',num2str(version),'\trajectory_data']);

%% Plotting
% clearvars -except maindir
clc
close all
load([maindir,'files_for_trajectory\version',num2str(version),'\trajectory_data'])

% figure
% hold on
% plot(socCheck_MLE)
% plot(socCheck_MF,'--')

% Define color variables via hex codes
Blue = '#0072BD';
Orange = '#D95319';
Yellow = '#EDB120';
Purple = '#7E2F8E';
Green = '#77AC30';
Cyan = '4DBEEE';
Adobo = 'A2142F';

% Grays
Black = '#000000';
DarkGray = '#868686';
LightGray = '#A6A6A6';
White = '#FFFFFF';
% Font size, line size, and line width.
font_size = 15;
line_size = 15;
line_width = 2;

% Position and Velocity
figure
t_layout = tiledlayout(2,2);        
t_layout.Padding = 'compact';
t_layout.TileSpacing = 'compact';
nexttile(1)
hold on
stairs(k*dT,r_MLE(1,:),'-','Linewidth',line_width,'Color',Blue)
stairs(k*dT,r_MF(1,:),'-','Linewidth',line_width,'Color',Orange)
ylabel('$x$ (m)','fontsize',font_size,'Interpreter','Latex')
xticks([0 dT*N/2 dT*N])
xlim('tight')
ylim('padded')
xticklabels('');
grid on
% subplot(223)
% hold on
% stairs(k*dT,r_MLE(2,:),'-','Linewidth',line_width,'Color',Blue)
% stairs(k*dT,r_MF(2,:),'-','Linewidth',line_width,'Color',Orange)
% ylabel('$Y$ (m)','fontsize',font_size,'Interpreter','Latex')
% grid on
nexttile(3)
hold on
stairs(k*dT,r_MLE(3,:),'-','Linewidth',line_width,'Color',Blue)
stairs(k*dT,r_MF(3,:),'-','Linewidth',line_width,'Color',Orange)
ylabel('$h$ (m)','fontsize',font_size,'Interpreter','Latex')
grid on
xlabel('Time (s)','fontsize',font_size,'Interpreter','Latex')
xticks([0 dT*N/2 dT*N])
xlim('tight')
ylim('padded')
nexttile(2)
hold on
stairs(k*dT,dot_r_MLE(1,:),'-','Linewidth',line_width,'Color',Blue)
stairs(k*dT,dot_r_MF(1,:),'-','Linewidth',line_width,'Color',Orange)
xticklabels('');
ylabel('$\dot{x}$ (m/s)','fontsize',font_size,'Interpreter','Latex')
grid on
xlim('tight')
ylim('padded')
lg = legend(gca,'Min. Err.','Min. Fuel');
lg.Location = 'best'; % <-- Legend placement with tiled layout
% subplot(324)
% hold on
% stairs(k*dT,dot_r_MLE(2,:),'-','Linewidth',line_width,'Color',Blue)
% stairs(k*dT,dot_r_MF(2,:),'-','Linewidth',line_width,'Color',Orange)
% ylabel('$\dot{Y}$ (m/s)','fontsize',font_size,'Interpreter','Latex')
% grid on
nexttile(4)
hold on
stairs(k*dT,dot_r_MLE(3,:),'-','Linewidth',line_width,'Color',Blue)
stairs(k*dT,dot_r_MF(3,:),'-','Linewidth',line_width,'Color',Orange)
ylabel('$\dot{h}$ (m/s)','fontsize',font_size,'Interpreter','Latex')
grid on
xlabel('Time (s)','fontsize',font_size,'Interpreter','Latex')
xticks([0 dT*N/2 dT*N])
xlim('tight')
ylim('padded')
savefig(['files_for_trajectory\version',num2str(version),'\trajectory'])
gcfig = gcf;
print2eps(gcfig,['files_for_trajectory\version',num2str(version),'\trajectory.eps'],16,9,'painters')
print2png(gcfig,['files_for_trajectory\version',num2str(version),'\trajectory.png'],16,9)
clear gcfig

% 3D Parametric Position Comparison
r_gs = linspace(0,c_gs.'*x0(1:3));
th_gs = linspace(0,2*pi);
[R_gs,T_gs] = meshgrid(r_gs,th_gs) ;
X_gs = R_gs.*cos(T_gs) + r_MF(1,end);
Y_gs = R_gs.*sin(T_gs) + r_MF(2,end);
Z_gs = -R_gs./c_gs(3);
[temp1,temp2] = meshgrid(linspace(min(r_MF(1,:))-delta_star_MF-0.2,max(r_MF(1,:))+delta_star_MF+0.2),linspace(min(r_MF(2,:))-delta_star_MF-0.2,max(r_MF(2,:))+delta_star_MF+0.2));

figure
hold on
plot3(r_MLE(1,:),r_MLE(2,:),r_MLE(3,:),'-o','LineWidth',2,'Color',Blue)
plot3(r_MF(1,:),r_MF(2,:),r_MF(3,:),'-o','LineWidth',2,'Color',Orange)
% plot3(r_MLE(1,:),r_MLE(2,:),r_MLE(3,:),'x','MarkerSize',6,'MarkerEdgeColor',Blue,'MarkerFaceColor',Blue)
% plot3(r_MF(1,:),r_MF(2,:),r_MF(3,:),'o','MarkerSize',4,'MarkerEdgeColor',Orange,'MarkerFaceColor',Orange)
% plot3(r_MF(1,1),r_MF(2,1),r_MF(3,1),'o','MarkerSize',6,'MarkerEdgeColor','#616161','MarkerFaceColor','k')
% plot3(r_MF(1,end),r_MF(2,end),r_MF(3,end),'x','MarkerSize',8,'Linewidth',3,'Color','k')
% plot3(rd(1),rd(2),0,'p','MarkerSize',8,'MarkerEdgeColor','k','MarkerFaceColor','#FFC107')
% surf(temp1,temp2,zeros(size(temp1)),'FaceAlpha',1,'LineStyle','-','FaceColor','#8D6E63','EdgeColor','#8D6E63')
% surf(X_gs,Y_gs,Z_gs,'FaceAlpha',0,'LineStyle','-','EdgeColor','#BDBDBD')
% xlim([min(r_MF(1,:))-delta_star_MF-0.2 max(r_MF(1,:))+delta_star_MF+0.2])
% ylim([min(r_MF(2,:))-delta_star_MF-0.2 max(r_MF(2,:))+delta_star_MF+0.2])
% zlim([min(r_MF(3,:))-0.2 max(r_MF(3,:))+delta_star_MF+0.2])
view(130,30)
zlim('padded')
xlim('padded')
ylim('padded')
grid on
axis equal
% limits = get(gca,'XLim');
% xlim([-0.1 limits(2)])
% title('MLE vs MF')
xlabel('$x$ (m)','Interpreter','Latex')
ylabel('$y$ (m)','Interpreter','Latex')
zlabel('$h$ (m)','Interpreter','Latex')
lg = legend(gca,'Min. Err.','Min. Fuel');
lg.Location = 'best'; % <-- Legend placement with tiled layout
lg_sz = lg.Position;
lg.Position = [0.6 0.3 lg.Position(3) lg.Position(4)];
savefig(['files_for_trajectory\version',num2str(version),'\3Dtrajectory'])
gcfig = gcf;
print2eps(gcfig,['files_for_trajectory\version',num2str(version),'\3Dtrajectory.eps'],9,14,'painters')
print2png(gcfig,['files_for_trajectory\version',num2str(version),'\3Dtrajectory.png'],9,14)
clear gcfig

% % 3D Parametric Position with thrust for MF solution
% minr = min(-r_MF(3,:));
% maxr = max(-r_MF(3,:));
% thr_scl = (maxr-minr)/10;
% maxTc = max(normTc_MF);
% figure
% hold on
% plot3(r_MF(1,:),r_MF(2,:),r_MF(3,:),'o','MarkerSize',4,'MarkerEdgeColor',Orange,'MarkerFaceColor',Orange)
% plot3(r_MF(1,1),r_MF(2,1),r_MF(3,1),'o','MarkerSize',6,'MarkerEdgeColor','#616161','MarkerFaceColor','k')
% plot3(r_MF(1,end),r_MF(2,end),r_MF(3,end),'x','MarkerSize',8,'Linewidth',3,'Color','k')
% plot3(rd(1),rd(2),0,'p','MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','#FFC107')
% for LV1 = 1:N
%     thr = thr_scl*(Tc_MF(:,LV1)/maxTc);
%     p0 = r_MF(:,LV1) - [thr(1); thr(2); -thr(3)];
%     pf = r_MF(:,LV1);
%     dp = pf-p0;
%     quiver3(p0(1),p0(2),p0(3),dp(1),dp(2),dp(3),'AutoScale',0,'MaxHeadSize',0.5,'Linewidth',1,'Color','#F44336');
% end
% xlim([min(r_MF(1,:))-delta_star_MF-0.2 max(r_MF(1,:))+delta_star_MF+0.2])
% ylim([min(r_MF(2,:))-delta_star_MF-0.2 max(r_MF(2,:))+delta_star_MF+0.2])
% zlim([min(r_MF(3,:))-0.2 max(r_MF(3,:))+0.2])
% grid on
% axis equal
% surf(temp1,temp2,zeros(size(temp1)),'FaceAlpha',1,'LineStyle','-','FaceColor','#8D6E63','EdgeColor','#8D6E63')
% surf(X_gs,Y_gs,Z_gs,'FaceAlpha',0,'LineStyle','-','EdgeColor','#BDBDBD')
% view(3)
% legend('Path','Start','Finish','Target','Thrust','Location','best')
% title('Minimum Fuel w/ Thrust')
% xlabel('X (m)','Interpreter','Latex')
% ylabel('Y (m)','Interpreter','Latex')
% zlabel('Altitude (m)','Interpreter','Latex')

% Constraints
figure
t_layout = tiledlayout(2,2);        
t_layout.Padding = 'compact';
t_layout.TileSpacing = 'compact';
nexttile(1) % Thrust magnitude constraint
hold on
grid on
stairs(k(1:end)*dT,[normTc_MLE normTc_MLE(end)],'-','Linewidth',line_width,'Color',Blue)
stairs(k(1:end)*dT,[normTc_MF normTc_MF(end)],'-','Linewidth',line_width,'Color',Orange)
plot(k(1:end)*dT,rho_min.*ones(size(k(1:end))),'--','Linewidth',line_width,'Color','#000000')
plot(k(1:end)*dT,rho_max.*ones(size(k(1:end))),'--','Linewidth',line_width,'Color','#000000')
yticks([0 12.5 25])
ylabel('$||T_{c}||$ (N)','fontsize',font_size,'Interpreter','Latex')
xticks([0 dT*N/2 dT*N])
xticklabels('');
xlim('tight')
ylim([0 rho_max])
% legend('MF','MLE','Interpreter','Latex','Location','Best')

nexttile(2) % Thrust pointing constraint
hold on
grid on
stairs(k(1:end)*dT,[theta_hat_MLE theta_hat_MLE(end)],'-','Linewidth',line_width,'Color',Blue)
stairs(k(1:end)*dT,[theta_hat_MF theta_hat_MF(end)],'-','Linewidth',line_width,'Color',Orange)
plot(k(1:end)*dT,theta.*ones(size(k(1:end))),'--','Linewidth',line_width,'Color','#000000')
% xlabel('Time (s)','fontsize',font_size,'Interpreter','Latex')
ylabel('$\hat{\theta}$ (deg)','fontsize',font_size,'Interpreter','Latex')
lg = legend(gca,'Min. Err.','Min. Fuel','Constraint');
% lg.Orientation = 'horizontal';
lg.Location = 'northeast'; % <-- Legend placement with tiled layout
xticks([0 dT*N/2 dT*N])
xticklabels('');
xlim('tight')
ylim('padded')

nexttile(3) % Glideslope constraint
hold on
grid on
stairs(k(1:end)*dT,GSd_MLE,'-','Linewidth',line_width,'Color',Blue)
stairs(k(1:end)*dT,GSd_MF,'-','Linewidth',line_width,'Color',Orange)
plot(k(1:end)*dT,gamma_gs.*ones(size(k(1:end))),'--','Linewidth',line_width,'Color','k')
xlabel('Time (s)','fontsize',font_size,'Interpreter','Latex')
ylabel('$\gamma$ (deg)','fontsize',font_size,'Interpreter','Latex')
xticks([0 dT*N/2 dT*N])
xlim('tight')
ylim([-10 100])
% legend('Min. Err.','Min. Fuel','Constraint','Interpreter','Latex','Location','Best')

nexttile(4) % Velocity constraint
hold on
grid on
plot(k(1:end)*dT,Vmax.*ones(size(k(1:end))),'--','Linewidth',line_width,'Color','#000000')
stairs(k(1:end)*dT,normV_MLE,'-','Linewidth',line_width,'Color',Blue)
stairs(k(1:end)*dT,normV_MF,'-','Linewidth',line_width,'Color',Orange)
xticks([0 dT*N/2 dT*N])
xlabel('Time (s)','fontsize',font_size,'Interpreter','Latex')
ylabel('$||\mathbf{v}^{\ast}||$ (m/s)','fontsize',font_size,'Interpreter','Latex')

savefig(['files_for_trajectory\version',num2str(version),'\constraints'])
gcfig = gcf;
print2eps(gcfig,['files_for_trajectory\version',num2str(version),'\constraints.eps'],16,9,'painters')
print2png(gcfig,['files_for_trajectory\version',num2str(version),'\constraints.png'],16,9)
clear gcfig
top_fig = gcf;
figure(top_fig);



%% MLE vs MF
% figure
% hold on 
% title('Optimization Results vs. Flight Time','Interpreter','Latex')
% yyaxis('left')
% plot(dT.*(N0:-1:N),delta_star_MLE,'-','Linewidth',line_width)
% xlabel('Flight time $k_{f}$ (s)','fontsize',font_size,'Interpreter','Latex')
% ylabel('Landing Error (m)','fontsize',font_size,'Interpreter','Latex')
% yyaxis('right')
% plot(dT.*(N0:-1:N),fval_MF,'-','Linewidth',line_width)
% ylabel('$||\mathbf{u}||_1$ (N)','fontsize',font_size,'Interpreter','Latex')
% grid on
% set(gca,'XMinorGrid','off','GridLineStyle','-','FontSize',line_size)
% % savefig('files_for_trajectory\FlightTimeVSMLEandMF')
% gcfig = gcf;
% exportgraphics(gcfig,'files_for_trajectory\FlightTimeVSMLEandMF.png','Resolution',300)
% clear gcfig



% delete(gcp('nocreate'))
disp('***Completed***')
