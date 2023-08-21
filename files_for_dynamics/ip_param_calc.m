% Title: ip_param_calc.m
% Author(s): William Elke III
% Date: 30-Nov-2020
% Description: Script with the purpose of calculating the mass a stiffness
% properties of the inverted pendulum of the CRQS.
%
% Updated 02-Jul-2023
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
clear
clc

% Add paths to necessary scripts
maindir = [pwd,'\']; % Make sure you are in the main directory first! NOT the dynamics directory!
addpath([maindir,'files_for_control'],...
            [maindir,'files_for_dynamics'],[maindir,'files_for_support'],...
            [maindir,'files_for_trajectory'],[maindir,'results'])

%% Define Constants and Initialize Variables
% Conversions
in2m = (0.0254)/(1); % [(m)/(in)] Convert from inches to meters.

% Constant parameters throughout all cases
m = 0.250; % [kg] Mass of inverted pendulum.
L = 1.6431; % [m] Length of inverted pendulum.
OR = (0.508/2)*in2m; % [m] Outer radius of inverted pendulum.
A = pi*(OR^2); % [m^2] Cross-secitonal area of rod.
sigma = m/(A*L); % [kg/(m^3)] Density.
I = (pi/4)*(OR^4) ; % [m^4] Second moment of area of a circlular cross section.
J = (pi/2)*(OR^4) ; % [m^4] Second polar moment of a circlular cross section.

% Specifications of our Polycarbonate rod case 1
% MatName = 'PolyCarb_flex_1';
% E = 2.2e9; % [Pa] Modulus of Elasticity.
% G = 700e6; % [Pa] Shear Modulus.
% num_bend_modes = 2; % [-] Number of bending modes considered.
% num_twist_modes = 0; % [-] Number of twisting modes considered.
% guesses = [5, 15].*(2*pi); % [rad/s] Initial guesses of the frequencies of the first 'num_modes' modes.
% m_end = 0; % [kg] Mass of end mass fixed to end of inverted pendulum

% Specifications of our Polycarbonate rod when it's artificially stiffened
% to be rigid. RIP case 1
% MatName = 'PolyCarb_rigid_1';
% E = 100e9; % [Pa] Rigid Modulus of Elasticity 
% G = 7e7; % [Pa] Shear Modulus 
% num_bend_modes = 1; % [-] Number of bending modes considered.
% num_twist_modes = 0; % [-] Number of twisting modes considered.
% guesses = 25.*(2*pi); % [rad/s] Initial guesses of the frequencies of the first 'num_modes' modes.
% m_end = 0; % [kg] Mass of end mass fixed to end of inverted pendulum

% % Specifications of our Polycarbonate rod case 2
% MatName = 'PolyCarb_flex_2';
% E = 2.2e9; % [Pa] Modulus of Elasticity.
% G = 700e6; % [Pa] Shear Modulus.
% num_bend_modes = 2; % [-] Number of bending modes considered.
% num_twist_modes = 0; % [-] Number of twisting modes considered.
% guesses = [5, 10, 25].*(2*pi); % [rad/s] Initial guesses of the frequencies of the first 'num_modes' modes.
% m_end = 0.05; % [kg] Mass of end mass fixed to end of inverted pendulum

% Specifications of our Polycarbonate rod when it's artificially stiffened
% to be rigid. RIP case 2
MatName = 'PolyCarb_rigid_2';
E = 100e9; % [Pa] Rigid Modulus of Elasticity 
G = 700e6; % [Pa] Shear Modulus 
num_bend_modes = 1; % [-] Number of bending modes considered.
num_twist_modes = 0; % [-] Number of twisting modes considered.
guesses = 25.*(2*pi); % [rad/s] Initial guesses of the frequencies of the first 'num_modes' modes.
m_end = 0.05; % [kg] Mass of end mass fixed to end of inverted pendulum

% Total number of flex modes
num_modes = num_bend_modes + num_twist_modes;

%% Solve for mode shapes of a pinned-free beam with end mass. See Section 11.5 of "Flexible Multibody Dynamics" by O.A. Bauchau

syms x y z r phi wn_sym beta_sym

eqn1 = ((E*I)^2 * beta_sym.^4) .* (tanh(beta_sym.*L) - tan(beta_sym.*L))...
        - (2 .* E*I .* beta_sym .* wn_sym.^2 * m_end .* tan(beta_sym.*L) .* tanh(beta_sym.*L)) == 0;

eqn2 = wn_sym == ((beta_sym*L)^2)*sqrt((E*I)/(sigma*A*(L^4)));

eqns = [eqn1, eqn2];

for LV1 = 1:num_bend_modes
    wn_guess = guesses(LV1);
    init_param = [wn_guess, (1/L)*sqrt(wn_guess/sqrt((E*I)/(sigma*A*(L^4))))];
    S = vpasolve(eqns,[wn_sym,beta_sym],init_param);
    wn_sol(LV1) = double(S.wn_sym);
    beta(LV1) = double(S.beta_sym);
end

wn_Hz_solv = double(wn_sol)./(2*pi) % Approximate bending mode natural frequencies

% b1-Bending (y)
PSI_v = [];
for LV1 = 1:num_bend_modes
    num = (m_end*(wn_sol(LV1)^2)*sin(beta(LV1)*L)) - (E*I*(beta(LV1)^3)*cos(beta(LV1)*L));
    den = (m_end*(wn_sol(LV1)^2)*sinh(beta(LV1)*L)) + (E*I*(beta(LV1)^3)*cosh(beta(LV1)*L));
    PSI_v = [PSI_v sin(beta(LV1)*x) - ( (num/den) * sinh(beta(LV1)*x) )]; % Transverse x basis functions
end

% b2-Bending (z)
PSI_w = PSI_v; % Bending modes the same in each direction

% Twist
PSI_theta = [];
for LV1 = 1:num_twist_modes
    PSI_theta = [PSI_theta (x/L)^(LV1+1)]; % Twist basis functions
end


% Define the limits of integration for the volume integral over the body of
% the IP
r_lb = 0; % Lower bound of indefinite integral in r, the radius
r_ub = OR; % Upper bound of indefinite integral in r, the radius
phi_lb = 0; % Lower bound of indefinite integral in phi, the angle of rotation
phi_ub = 2*pi; % Upper bound of indefinite integral in phi, the angle of rotation
x_lb = 0; % Lower bound of indefinite integral in x
x_ub = L; % Upper bound of indefinite integral in x

y_lb = -sqrt((OR^2) - (z^2)); % Lower bound of indefinite integral in y
y_ub = sqrt((OR^2) - (z^2)); % Upper bound of indefinite integral in y
z_lb = -OR; % Lower bound of indefinite integral in z
z_ub = OR; % Upper bound of indefinite integral in z

%% Inverted Pendulum Mass Properties
% Define Psi (see Eq. (14))
P11 = PSI_v;
P22 = PSI_w;
P12 = zeros(1,size(P22,2));
P21 = zeros(1,size(P11,2));
P31 = -y.*diff(PSI_v,x,1);
P32 = -z.*diff(PSI_w,x,1);

if num_twist_modes > 0
    P13 = -z.*PSI_theta;
    P23 = y.*PSI_theta;
    P33 = zeros(1,size(P13,2));
    PSI = [P11 P12 P13;
        P21 P22 P23;
        P31 P32 P33];
else
    PSI = [P11 P12;
        P21 P22;
        P31 P32];
end

% Calculate mass properties of the undeformed IP. Assume constant density throughout the IP
m_rod = double(int(int(int(sigma*r,r,r_lb,r_ub),phi,phi_lb,phi_ub),x,x_lb,x_ub)); % [kg] Mass of IP
const.m_B = m_rod + m_end; % [kg] Mass of IP and end mass
const.r_bm_b = (1/const.m_B)*((m_rod.*[0 0 -(L/2)]') + (m_end.*[0 0 -L]')); % [m m m]' Position vector from the pin location of the IP to the CM of the undeformed IP and end mass resolved in the b frame
const.r_bm_b_X = CrossOp(const.r_bm_b); % Apply cross operator
r_em_e = transpose([r*cos(phi) r*sin(phi) x]); % Position vector to e from m resolved in the r-phi-z frame
r_em_e_X = CrossOp(r_em_e); % Apply cross operator
r_em_b = transpose([y z x]); % Position vector from to e from m resolved in the Frame b. See Eq. (11).
r_em_b_X = CrossOp(r_em_b); % Apply cross operator
c_Bm_b = double(int(int(int(sigma*r*r_em_e,r,r_lb,r_ub),phi,phi_lb,phi_ub),x,x_lb,x_ub)) + (m_end.*[0 0 -L]'); % [kg*m] First moment of mass of the IP about the pinned location m
const.c_Bm_b_X = CrossOp(c_Bm_b);
const.J_Bm_b = double(int(int(int(-sigma*r*r_em_e_X*r_em_e_X,r,r_lb,r_ub),phi,phi_lb,phi_ub),x,x_lb,x_ub)) + (m_end.*CrossOp([0 0 -L]')*CrossOp([0 0 -L]')); % [kg*(m^2)] Second moment of mass of the IP about the pinned location m

CM_loc_percent = (abs(const.r_bm_b(3))./L)*100 % Location of CM from pin point in percent
w_grav = ( sqrt(9.81/abs(const.r_bm_b(3))) )*(1/(2*pi)) % Gravitational "frequency"

% Calculate mass properties for deforming IP
const.P_e = double(int(int(int(sigma*PSI,y,y_lb,y_ub),z,z_lb,z_ub),x,x_lb,x_ub)); % Modal momentum
const.H_e = double(int(int(int(sigma*r_em_b_X*PSI,y,y_lb,y_ub),z,z_lb,z_ub),x,x_lb,x_ub)); % Modal angular momentum
const.M_e = double(int(int(int(sigma.*((PSI.')*PSI),y,y_lb,y_ub),z,z_lb,z_ub),x,x_lb,x_ub)); % Modal mass
disp('M = ')
disp(const.M_e)

%% Stiffness Matrix
% Define the integrand used for stiffness matrix calculation. See Eq. (10).
ddPSI_v = diff(PSI_v,x,2);
ddPSI_w = diff(PSI_w,x,2);

if num_twist_modes > 0
    dPSI_theta = diff(PSI_theta,x,1);
    K_int = blkdiag(E*I.*(ddPSI_v'*ddPSI_v), E*I.*(ddPSI_w'*ddPSI_w), G*J.*(dPSI_theta.'*dPSI_theta));
else
    K_int = blkdiag(E*I.*(ddPSI_v'*ddPSI_v), E*I.*(ddPSI_w'*ddPSI_w));
end

% Calculate the stiffness matrix K_e via definite integral
const.K_e = double(int(K_int,x,x_lb,x_ub)); % Stiffness matrix
disp('K = ')
disp(const.K_e)

%% Damping Matrix
wn = sqrt(eig(const.M_e\const.K_e));
if num_twist_modes > 0
    wn = [sort(wn(1:num_bend_modes))', sort(wn(num_bend_modes+1:2*num_bend_modes))', sort(wn(2*num_bend_modes+1:2*num_bend_modes+num_twist_modes))']';
else
    wn = [sort(wn(1:num_bend_modes))', sort(wn(num_bend_modes+1:2*num_bend_modes))']';
end

zeta = 0.2/100; % [-] Damping for each flex mode.
const.D_e = 2*zeta.*diag(wn);

% zeta1 = 0.05;
% zeta2 = 0.2;
% rayleigh_coeffs = [wn(1)^(-1) wn(1); wn(2)^(-1) wn(2)]\[2*zeta1; 2*zeta2];
% alpha_damp = rayleigh_coeffs(1)
% beta_damp = rayleigh_coeffs(2)
% const.D_e = alpha_damp.*const.M_e + beta_damp.*const.K_e;

disp('D =')
disp(const.D_e)

%% Calculate PSI at tip (see Eq. (36))
PSI_v_t = double(subs(PSI_v,x,L)); % Transverse y basis functions
PSI_w_t = PSI_v_t; % Transverse z basis functions
P11_t = PSI_v_t;
P22_t = PSI_w_t;
P12_t = zeros(1,size(P22_t,2));
P21_t = zeros(1,size(P11_t,2));
P31_t = zeros(1,size(P11_t,2));
P32_t = zeros(1,size(P22_t,2));

if num_twist_modes > 0
    PSI_theta_t = double(subs(PSI_theta,x,L)); % Twist basis functions
    P13_t = zeros(1,size(PSI_theta_t,2));
    P23_t = zeros(1,size(PSI_theta_t,2));
    P33_t = zeros(1,size(PSI_theta_t,2));

    const.PSI_t = [P11_t P12_t P13_t;
        P21_t P22_t P23_t;
        P31_t P32_t P33_t];
else

    const.PSI_t = [P11_t P12_t;
        P21_t P22_t;
        P31_t P32_t];

end

%% Save variables for future use
% Uncomment the following line to save new .mat files or overwrite existing ones 
% save([maindir,'files_for_dynamics\IP_',MatName],'const')

%% Plot basis functions along length of beam 
disp('wn_Hz = ')
disp(wn./(2*pi))
disp('wd_Hz = ')
disp((wn.*sqrt(1-zeta^2))./(2*pi))
vary = 0:1/1000:L;

dPSI_v = diff(PSI_v,x,1);
dPSI_w = diff(PSI_w,x,1);

figure('Name','Shape')
hold on
for LV1 = 1:length(PSI_v)
    plot(vary,subs(PSI_v(LV1),x,vary),DisplayName=['Mode ',num2str(LV1)])
end
xlabel('Length along rod (m)')
ylabel('Deflection')
title('Bending Mode Shape')
legend('Location','Best')

figure('Name','Slope')
hold on
for LV1 = 1:length(dPSI_v)
    plot(vary,subs(dPSI_v(LV1),x,vary),DisplayName=['Mode ',num2str(LV1)])
end
xlabel('Length along rod (m)')
ylabel('Slope')
title('Bending Mode Slope')
legend('Location','Best')

if num_twist_modes > 0

    dPSI_theta = diff(PSI_theta,x,1);

    figure('Name','Shape')
    hold on
    for LV1 = 1:length(PSI_theta)
        plot(vary,subs(PSI_theta(LV1),x,vary),DisplayName=['Mode ',num2str(LV1)])
    end
    xlabel('Length along rod (m)')
    ylabel('Deflection')
    title('Twisting Mode Shape')
    legend('Location','Best')
    top_fig = gcf;

    figure('Name','Slope')
    hold on
    for LV1 = 1:length(dPSI_theta)
        plot(vary,subs(dPSI_theta(LV1),x,vary),DisplayName=['Mode ',num2str(LV1)])
    end
    xlabel('Length along rod (m)')
    ylabel('Slope')
    title('Twisting Mode Slope')
    legend('Location','Best')

end









