% Title: plotscript.m
% Author(s): William Elke III
% Date: 2-Feb-2021
% Description: Script with the purpose of plotting the data of the
% quadrotor helicopter system simulated in Main_Compile_File_vX.m
%
% Updated 11-Mar-2021
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
clearvars fig_*
toggle_save = 0; % Toggle to save figures. 0 = don't save, 1 = save

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Quadcopter Translation Plots
if ~exist('fig_quadtrans','var')
    fig_quadtrans = figure('Name','QuadTrans');
else
    figure(fig_quadtrans);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%
% Quadcopter position
subplot(321) % Plot of X-Position vs time
hold on
% plot(t,e(1,:))
plot(t,r_qi_i(1,:));
% stairs(t_traj,x_d(1,:),'LineStyle','-.','Color',Black)
% xlabel('Time (s)');
ylabel('$X$ (m)');
title('Quadcopter Position')

subplot(323) % Plot of Y-Position vs time
hold on
% plot(t,e(2,:))
plot(t,r_qi_i(2,:));
% stairs(t_traj,x_d(2,:),'LineStyle','-.','Color',Black)
% xlabel('Time (s)');
ylabel('$Y$ (m)');

subplot(325) % Plot of Altitude vs time
hold on
% plot(t,e(3,:))
plot(t,-r_qi_i(3,:));
% stairs(t_traj,-x_d(3,:),'LineStyle','-.','Color',Black)
xlabel('Time (s)');
ylabel('Alt. (m)');

%%%%%%%%%%%%%%%%%%%%%%%%%%
% Quadcopter velocity
subplot(322) % Plot of X-Position vs time
hold on
% plot(t,e(9,:))
plot(t,v_qi_i(1,:));
% stairs(t_traj,x_d(9,:),'LineStyle','--','Color',Black)
% xlabel('Time (s)');
ylabel('$\dot{X}$ (m/s)');
title('Quadcopter Velocity')

subplot(324) % Plot of Y-Position vs time
hold on
% plot(t,e(10,:))
plot(t,v_qi_i(2,:));
% stairs(t_traj,x_d(10,:),'LineStyle','--','Color',Black)
% xlabel('Time (s)');
ylabel('$\dot{Y}$ (m/s)');

subplot(326) % Plot of Altitude vs time
hold on
% plot(t,e(11,:))
plot(t,-v_qi_i(3,:));
% stairs(t_traj,x_d(11,:),'LineStyle','--','Color',Black)
xlabel('Time (s)');
ylabel('Alt. Rate (m/s)');

%%%%%%%%%%%%%%%%%%%%%%%%%%
% Save figure
if toggle_save == 1
savefig(['.\results\',version_name,'\QuadTransPlot'])
gcfig = gcf;
exportgraphics(gcfig,['.\results\',version_name,'\fig_quad_trans.png'],'Resolution',300)
clear gcfig
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Quadcopter Rotation Plots
if ~exist('fig_quadatt','var')
    fig_quadatt = figure('Name','QuadAtt');
else
    figure(fig_quadatt);
end
sgtitle('Inertial-Relative Quad Rot')

%%%%%%%%%%%%%%%%%%%%%%%%%%
% Quadcopter attitude
subplot(321) % Plot of Roll Angle vs time
hold on
plot(t,quad_phi);
% stairs(t_traj,x_d(4,:),'LineStyle','--')
% xlabel('Time (s)');
ylabel('Roll (deg)');
title('3-2-1 Euler')

subplot(323) % Plot of Pitch Angle vs time
hold on
plot(t,quad_theta);
% stairs(t_traj,x_d(5,:),'LineStyle','--')
% xlabel('Time (s)');
ylabel('Pitch (deg)');

subplot(325) % Plot of Yaw Angle vs time
hold on
plot(t,quad_psi);
% stairs(t_traj,x_d(6,:),'LineStyle','--')
xlabel('Time (s)');
ylabel('Yaw (deg)');

%%%%%%%%%%%%%%%%%%%%%%%%%%
% Quadcopter angular velocity
subplot(322) % Plot of Roll Angle Rate vs time
plot(t,w_qi_q(1,:));
hold on
% xlabel('Time (s)');
ylabel('$\mathbf{\omega}_1$ (deg/s)');
title('$\mathbf{\omega}^{qi}_{q}$')

subplot(324) % Plot of Pitch Angle Rate vs time
plot(t,w_qi_q(2,:));
hold on
% xlabel('Time (s)');
ylabel('$\mathbf{\omega}_2$ (deg/s)');

subplot(326) % Plot of Yaw Angle Rate vs time
plot(t,w_qi_q(3,:));
hold on
xlabel('Time (s)');
ylabel('$\mathbf{\omega}_3$ (deg/s)');

%%%%%%%%%%%%%%%%%%%%%%%%%%
% Save figure
if toggle_save == 1
savefig(['.\results\',version_name,'\QuadAttPlot'])
gcfig = gcf;
exportgraphics(gcfig,['.\results\',version_name,'\fig_quad_rot.png'],'Resolution',300)
clear gcfig
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Inverted Pendulum Rotation Plots
if ~exist('fig_IPatt','var')
    fig_IPatt = figure('Name','IPAtt');
else
    figure(fig_IPatt);
end
sgtitle('Quad-Relative IP Rot')

%%%%%%%%%%%%%%%%%%%%%%%%%%
% IP attitude
subplot(321) % Plot of Roll Angle vs time
plot(t,IP_phi);
hold on
% xlabel('Time (s)');
ylabel('Roll (deg)');
title('3-2-1 Euler')

subplot(323) % Plot of Pitch Angle vs time
plot(t,IP_theta);
hold on
% xlabel('Time (s)');
ylabel('Pitch (deg)');

subplot(325) % Plot of Yaw Angle vs time
plot(t,IP_psi);
hold on
xlabel('Time (s)');
ylabel('Yaw (deg)');

%%%%%%%%%%%%%%%%%%%%%%%%%%
% IP angular velocity
subplot(322) % Plot of Roll Angle Rate vs time
plot(t,w_bq_b(1,:));
hold on
% xlabel('Time (s)');
ylabel('$\mathbf{\omega}_1$ (deg/s)');
title('$\mathbf{\omega}^{bq}_{b}$')

subplot(324) % Plot of Pitch Angle Rate vs time
plot(t,w_bq_b(2,:));
hold on
% xlabel('Time (s)');
ylabel('$\mathbf{\omega}_2$ (deg/s)');

subplot(326) % Plot of Yaw Angle Rate vs time
plot(t,w_bq_b(3,:));
hold on
xlabel('Time (s)');
ylabel('$\mathbf{\omega}_3$ (deg/s)');

%%%%%%%%%%%%%%%%%%%%%%%%%%
% Save figure
if toggle_save == 1
savefig(['.\results\',version_name,'\IPAttPlot'])
gcfig = gcf;
exportgraphics(gcfig,['.\results\',version_name,'\fig_IP_rot.png'],'Resolution',300)
clear gcfig
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Hanging Pendulum Rotation Plots
if ~exist('fig_HPatt','var')
    fig_HPatt = figure('Name','HPAtt');
else
    figure(fig_HPatt);
end
sgtitle('Quad-Relative HP Rot')

%%%%%%%%%%%%%%%%%%%%%%%%%%
% HP attitude
subplot(321) % Plot of Roll Angle vs time
plot(t,HP_phi);
hold on
% xlabel('Time (s)');
ylabel('Roll (deg)');

title('3-2-1 Euler')
subplot(323) % Plot of Pitch Angle vs time
plot(t,HP_theta);
hold on
% xlabel('Time (s)');
ylabel('Pitch (deg)');

subplot(325) % Plot of Yaw Angle vs time
plot(t,HP_psi);
hold on
xlabel('Time (s)');
ylabel('Yaw (deg)');

%%%%%%%%%%%%%%%%%%%%%%%%%%
% HP angular velocity
subplot(322) % Plot of Roll Angle Rate vs time
plot(t,w_hq_h(1,:));
hold on
% xlabel('Time (s)');
ylabel('$\mathbf{\omega}_1$ (deg/s)');
title('$\mathbf{\omega}^{sq}_{s}$')

subplot(324) % Plot of Pitch Angle Rate vs time
plot(t,w_hq_h(2,:));
hold on
% xlabel('Time (s)');
ylabel('$\mathbf{\omega}_2$ (deg/s)');

subplot(326) % Plot of Yaw Angle Rate vs time
plot(t,w_hq_h(3,:));
hold on
xlabel('Time (s)');
ylabel('$\mathbf{\omega}_3$ (deg/s)');

%%%%%%%%%%%%%%%%%%%%%%%%%%
% Save figure
if toggle_save == 1
savefig(['.\results\',version_name,'\HPAttPlot'])
gcfig = gcf;
exportgraphics(gcfig,['.\results\',version_name,'\fig_HP_rot.png'],'Resolution',300)
clear gcfig
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Inverted Pendulum Tip Motion
if ~exist('fig_TipDef','var')
    fig_TipDef = figure('Name','TipDef');
else
    figure(fig_TipDef);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%
% IP tip deflection
subplot(321)
hold on
plot(t,tip_def(1,:));
title('Tip Deflection')
ylabel('X (mm)');

subplot(323)
plot(t,tip_def(2,:));
hold on
ylabel('Y (mm)');

subplot(325)
hold on
plot(t,tip_def(3,:));
ylabel('Z (mm)');

%%%%%%%%%%%%%%%%%%%%%%%%%%
% IP tip rate of deflection
subplot(322)
hold on
plot(t,tip_rate(1,:));
ylabel('X rate (mm/s)');
title('Tip Def. Rate')

subplot(324)
hold on
plot(t,tip_rate(2,:));
ylabel('Y rate (mm/s)');

subplot(326)
plot(t,tip_rate(3,:));
hold on
xlabel('Time (s)');
ylabel('Z rate (mm/s)');

%%%%%%%%%%%%%%%%%%%%%%%%%%
% Save figure
if toggle_save == 1
savefig(['.\results\',version_name,'\TipDeflectionPlot'])
gcfig = gcf;
exportgraphics(gcfig,['.\results\',version_name,'\fig_IP_tip_def.png'],'Resolution',300)
clear gcfig
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Checks
if ~exist('fig_checks','var')
    fig_checks = figure('Name','Checks');
else
    figure(fig_checks);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%
% Energy, which should be constant when damping and control forces are zero.
subplot(331) % Plot of total energy vs time (should be constant)
plot(t,E);
hold on
title('Energy Check')
xlabel('Time (s)');
ylabel('$E$ (J)');

subplot(334) % Plot of normalized change in energy vs time (should be zero)
plot(t,(E-E(1))./E(1));
hold on
xlabel('Time (s)');
ylabel('$(E - E(0))/E(0)$ (J)');
% ylim([-1 1]);



%%%%%%%%%%%%%%%%%%%%%%%%%%
% DCM checks
subplot(332)
plot(t,quad_DCM_check);
hold on
xlabel('Time (s)');
ylabel('det$(C_{qi}) - 1$');
title('DCM Check')

subplot(335)
plot(t,IP_DCM_check);
hold on
xlabel('Time (s)');
ylabel('det$(C_{bi}) - 1$');

subplot(338)
plot(t,HP_DCM_check);
hold on
xlabel('Time (s)');
ylabel('det$(C_{si}) - 1$');

%%%%%%%%%%%%%%%%%%%%%%%%%%
% Constraint checks
subplot(333) 
plot(t,IP_constraint_check)
hold on
title('Constraint Checks')
xlabel('Time (s)');
ylabel('IP');

subplot(336)
plot(t,HP_constraint_check)
hold on
% title('Energy Check')
xlabel('Time (s)');
ylabel('HP');

%%%%%%%%%%%%%%%%%%%%%%%%%%
% Save figure
if toggle_save == 1
savefig(['.\results\',version_name,'\ChecksPlot'])
gcfig = gcf;
exportgraphics(gcfig,['.\results\',version_name,'\fig_checks.png'],'Resolution',300)
clear gcfig
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Control effort
if ~exist('fig_effort','var')
    fig_effort = figure('Name','Effort');
else
    figure(fig_effort);
end
sgtitle('Control Effort')

% Hover
% plot(t,const.bar_u.*ones(size(t)),'LineStyle','--')

% % Motor saturation
% if const.motor_UL ~= inf
%     plot(t,const.motor_UL.*ones(size(t)),'LineStyle','--')
% end

eff_LL = const.motor_LL*radps2rpm;
eff_UL = const.motor_UL*radps2rpm;

%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control effort of motor 1
subplot(222)
hold on
% stairs(t,u(1,:),'LineStyle','--','Color',Black);
plot(t,u_true(1,:).*radps2rpm,'LineStyle','-');
ylim([eff_LL eff_UL])
ylabel('Effort (rpm)');
title('Motor 1')

%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control effort of motor 2
subplot(223)
hold on
% stairs(t,u(2,:),'LineStyle','--','Color',Black);
plot(t,u_true(2,:).*radps2rpm,'LineStyle','-');
ylim([eff_LL eff_UL])
xlabel('Time (s)');
ylabel('Effort (rpm)');
title('Motor 2')

%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control effort of motor 3
subplot(221)
hold on
% p1 = stairs(t,u(3,:),'LineStyle','--','Color',Black);
p2 = plot(t,u_true(3,:).*radps2rpm,'LineStyle','-');
ylim([eff_LL eff_UL])
ylabel('Effort (rpm)');
title('Motor 3')
% legend([p1, p2],{'Desired','True'},'Location','Best','fontsize',font_size_1)

%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control effort of motor 4
subplot(224)
hold on
% stairs(t,u(4,:),'LineStyle','--','Color',Black);
plot(t,u_true(4,:).*radps2rpm,'LineStyle','-');
ylim([eff_LL eff_UL])
xlabel('Time (s)');
ylabel('Effort (rpm)');
title('Motor 4')

%%%%%%%%%%%%%%%%%%%%%%%%%%
if toggle_save == 1
    savefig(['.\results\',version_name,'\ControlEffortPlot'])
    gcfig = gcf;
    exportgraphics(gcfig,['.\results\',version_name,'\fig_effort.png'],'Resolution',300)
    clear gcfig
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% top_fig
top_fig = gcf;
figure(top_fig);
