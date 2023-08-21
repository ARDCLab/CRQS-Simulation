% Title: plotscript.m
% Author(s): William Elke III
% Date: 2-Feb-2021
% Description: Script with the purpose of plotting the data of the
% quadrotor helicopter system simulated in compile.m
%
% Updated 11-Mar-2021
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%  clearvars fig_*
%  close all

legend_var = [];

version = 2;

for LV_plot = 1:4
    if LV_plot == 4
        config_name = 'RIP';
    elseif LV_plot == 3
        config_name = 'RIP+HP';
    elseif LV_plot == 2
        config_name = 'FIP';
    elseif LV_plot == 1
        config_name = 'FIP+HP';
    end


    if LV_plot == 1
        clearvars -except config_name legend_var LV_plot version
        close all
        clc
    else
        clearvars -except fig_* config_name legend_var LV_plot version
    end
    load(['.\results\version',num2str(version),'\',config_name,'\sim_data_',config_name,'.mat']) % Load previously existing data for post-processing. Comment out "Simulation" section
    if ~exist(['.\results\co-plot\version',num2str(version)])
        mkdir(['.\results\co-plot\version',num2str(version)])
    end

    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Post Processing
    disp('Starting post processing...')
    postprocessing

    %% Define color variables via hex codes
%         Blue = '#1f77b4';
%         Orange = '#ff7f0e';
%         Green = '#2ca02c';
%         Red = '#d62728';
%         Purple = '#9467bd';
%         Brown = '#8c564b';
%         Pink = '#e377c2';
%         Gray =  '#7f7f7f';
%         YellowGreen = '#bcbd22';
%         Cyan = '#17becf';
%         Yellow = '#EDB120';

    Blue = '#0072BD';
    Orange = '#D95319';
    Yellow = '#EDB120';
    Purple = '#7E2F8E';
    Green = '#77AC30';
    Cyan = '4DBEEE';
    Adobo = 'A2142F';

    Black = '#000000';

    if strcmp(config_name,'RIP+HP')
        line_style = '-';
        color1 = Orange;
        plot_HP = 1;
        legend_name = 'RIP+HP';
    elseif strcmp(config_name,'FIP+HP')
        line_style = '-';
        color1 = Purple;
        plot_HP = 1;
        legend_name = 'FIP+HP';
    elseif strcmp(config_name,'FIP')
        line_style = '-';
        color1 = Yellow;
        plot_HP = 0;
        legend_name = 'FIP';
    else % strcmp(config_name,'RIP')
        line_style = '-';
        color1 = Blue;
        plot_HP = 0;
        legend_name = 'RIP';
    end

    if LV_plot == 4
        save_fig = 1;
    else
        save_fig = 0;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Quadcopter Plots
    if ~exist('fig_pos','var')
        fig_pos = figure('Name','pos');
        t_layout = tiledlayout(2,3);        
        t_layout.Padding = 'compact';
        t_layout.TileSpacing = 'compact';
    else
        figure(fig_pos);
    end

    nexttile(2)
    hold on
    % plot(t,e(2,:),'LineStyle',line_style,'Color',color1)
    plot_plot = plot(t,r_qi_i(2,:),'DisplayName',legend_name,'LineStyle',line_style,'Color',color1);
    % xlabel('Time (s)');
    ylabel('$y$ (m)');
    xticklabels('');
    if save_fig == 1
        plot_plot = stairs(t_traj,x_d(2,:),'LineStyle','--','Color',Black);
    end
    hold off
    nexttile(3) % Plot of Altitude vs time
    hold on
    % plot(t,e(3,:),'LineStyle',line_style,'Color',color1)
    plot_plot = plot(t,-r_qi_i(3,:),'DisplayName',legend_name,'LineStyle',line_style,'Color',color1);
%     xlabel('Time (s)');
    xticklabels('');
    ylabel('$h$ (m)');
    if save_fig == 1
        plot_plot = stairs(t_traj,-x_d(3,:),'LineStyle','--','Color',Black);
    end
    hold off
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    % Quadcopter velocity
    nexttile(4) % Plot of X-Position vs time
    hold on
    % plot(t,e(9,:),line_style,'Color',color1)
    plot_plot = plot(t,v_qi_i(1,:),'DisplayName',legend_name,'LineStyle',line_style,'Color',color1);
    xlabel('Time (s)');
    ylabel('$\dot{x}$ (m/s)');
%     title('Quadcopter Velocity')
    if save_fig == 1
        plot_plot = stairs(t_traj,x_d(9,:),'DisplayName','$\mathbf{x}^{d}$','LineStyle','--','Color',Black);
    end
    hold off

    nexttile(5) % Plot of Y-Position vs time
    hold on
    % plot(t,e(10,:),line_style,'Color',color1)
    plot_plot = plot(t,v_qi_i(2,:),'DisplayName',legend_name,'LineStyle',line_style,'Color',color1);
    xlabel('Time (s)');
    ylabel('$\dot{y}$ (m/s)');
    if save_fig == 1
        plot_plot = stairs(t_traj,x_d(10,:),'LineStyle','--','Color',Black);
    end
    hold off

    nexttile(6) % Plot of Altitude vs time
    hold on
    % plot(t,e(11,:),line_style,'Color',color1)
    plot_plot = plot(t,-v_qi_i(3,:),'DisplayName',legend_name,'LineStyle',line_style,'Color',color1);
    xlabel('Time (s)');
    ylabel('$\dot{h}$ (m/s)');
    if save_fig == 1
        plot_plot = stairs(t_traj,-x_d(11,:),'LineStyle','--','Color',Black);
    end
    hold off

    %%%%%%%%%%%%%%%%%%%%%%%%%%
    % Quadcopter position
    nexttile(1) % Plot of X-Position vs time
    hold on
    % plot(t,e(1,:),'LineStyle',line_style,'Color',color1)
    plot_plot = plot(t,r_qi_i(1,:),'DisplayName',legend_name,'LineStyle',line_style,'Color',color1);
    legend_var = [plot_plot legend_var];
    % xlabel('Time (s)');
    ylabel('$x$ (m)');
    xticklabels('');
%     title('Quadcopter Position')
    if save_fig == 1
        plot_plot = stairs(t_traj,x_d(1,:),'DisplayName','$\mathbf{x}^{d}$','LineStyle','--','Color',Black);
        legend_var = [legend_var plot_plot];
        lg = legend(gca,legend_var);
        lg.Orientation = 'horizontal';
        lg.Layout.Tile = 'South'; % <-- Legend placement with tiled layout
    end
    hold off


    %%%%%%%%%%%%%%%%%%%%%%%%%%
    % Save figure
    if save_fig == 1
%         fig = gcf;
%         fig.Position(3) = fig.Position(3) + 250;
%         disp('Please adjust location of legend, then press enter...')
%         pause
        savefig(['.\results\co-plot\version',num2str(version),'\PosPlot'])
        gcfig = gcf;
        print2eps(gcfig,['.\results\co-plot\version',num2str(version),'\fig_pos.eps'],16,9,'painters');
        print2png(gcfig,['.\results\co-plot\version',num2str(version),'\fig_pos.png'],16,9);
        clear gcfig
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%
    % Quadcopter attitude
    if ~exist('fig_att','var')
        fig_att = figure('Name','att');
        t_layout = tiledlayout(1,3);        
        t_layout.Padding = 'compact';
        t_layout.TileSpacing = 'compact';
    else
        figure(fig_att);
    end

    clear ylimits
    ylimits = [];

    nexttile(2)
    hold on
    plot_plot = plot(t,quad_theta,'DisplayName',legend_name,'LineStyle',line_style,'Color',color1);
    ax_quad_pitch = gca;
    % stairs(t_traj,x_d(5,:),'LineStyle','-.')
    xlabel('Time (s)');
    ylabel('$\theta$ (deg)');
    ylimits = [ylimits; get(gca,'Ylim')];
%     title('Quadcopter Attitude')

    nexttile(1)
    hold on
    plot_plot = plot(t,quad_phi,'DisplayName',legend_name,'LineStyle',line_style,'Color',color1);
    ax_quad_roll = gca;
    % stairs(t_traj,x_d(4,:),'LineStyle','-.')
    xlabel('Time (s)');
    ylabel('$\phi$ (deg)');
    ylimits = [ylimits; get(gca,'Ylim')];



    nexttile(3)
    hold on
    plot_plot = plot(t,quad_psi,'DisplayName',legend_name,'LineStyle',line_style,'Color',color1);
    ax_quad_yaw = gca;
    % stairs(t_traj,x_d(6,:),'LineStyle','-.')
    xlabel('Time (s)');
    ylabel('$\psi$ (deg)');
%     ylimits = [ylimits; get(gca,'Ylim')];


    if save_fig == 1
        set([ax_quad_pitch,ax_quad_roll],'Ylim',[min(ylimits(:,1)), max(ylimits(:,2))]);
        fig_pos_size = get(fig_pos,'Position');
        set(gcf,'Position',[fig_pos_size(1) fig_pos_size(2) fig_pos_size(3) fig_pos_size(4)*(3/5)])
        % set([ax_quad_pitch,ax_quad_roll,ax_quad_yaw],'Ylim',[-4,4]);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%
    % Save figure
    if save_fig == 1
        h = get(gca,'Children');
        lg = legend(gca,h(:));
        lg.Orientation = 'horizontal';
        lg.Layout.Tile = 'South'; % <-- Legend placement with tiled layout
        set(gca,'Children',h(end:-1:1))
        lg = legend(gca,h(:));
        set(gca,'Children',h(1:end))
%         fig = gcf;
%         fig.Position(3) = fig.Position(3) + 250;
%         disp('Please adjust location of legend, then press enter...')
%         pause
        savefig(['.\results\co-plot\version',num2str(version),'\AttPlot'])
        gcfig = gcf;
        print2eps(gcfig,['.\results\co-plot\version',num2str(version),'\fig_att.eps'],16,6,'painters');
        print2png(gcfig,['.\results\co-plot\version',num2str(version),'\fig_att.png'],16,6);
        clear gcfig
    end

    %%
    if ~exist('fig_feats','var')
        fig_feats = figure('Name','Features');
        t_layout = tiledlayout(2,3);        
        t_layout.Padding = 'compact';
        t_layout.TileSpacing = 'compact';
    else
        figure(fig_feats);
    end

    clear ylimits
    ylimits = [];
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    % IP attitude
    nexttile(1)
    hold on
    fig_alpha_ip = plot(t,alpha_ip,'DisplayName',legend_name,'LineStyle',line_style,'Color',color1);
    ax_alpha_ip = gca;
    % xlabel('Time (s)');
    ylabel('$\alpha_{ip}$ (deg)');
%     title('IP Orientation')
    xticklabels('');
    ylimits = [ylimits; get(gca,'Ylim')];

    nexttile(4)
    hold on
    fig_beta_ip = plot(t,beta_ip,'DisplayName',legend_name,'LineStyle',line_style,'Color',color1);
    ax_beta_ip = gca;
    xlabel('Time (s)');
    ylabel('$\beta_{ip}$ (deg)');
    ylimits = [ylimits; get(gca,'Ylim')];

        set([ax_alpha_ip,ax_beta_ip],'Ylim',[min(ylimits(:,1)), max(ylimits(:,2))]);
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    % HP attitude
    clear ylimits
    ylimits = [];
    if contains(config_name,'HP')
        nexttile(2)
        hold on
        fig_alpha_hp = plot(t,alpha_hp,'DisplayName',legend_name,'LineStyle',line_style,'Color',color1);
        ax_alpha_hp = gca;
        % xlabel('Time (s)');
        ylabel('$\alpha_{hp}$ (deg)');
        ylimits = [ylimits; get(gca,'Ylim')];
        xticklabels('');
%         title('HP Orientation')

        nexttile(5)
        hold on
        fig_beta_hp = plot(t,beta_hp,'DisplayName',legend_name,'LineStyle',line_style,'Color',color1);
        ax_beta_hp = gca;
        xlabel('Time (s)');
        ylabel('$\beta_{hp}$ (deg)');
        ylimits = [ylimits; get(gca,'Ylim')];
            set([ax_alpha_hp,ax_beta_hp],'Ylim',[min(ylimits(:,1)), max(ylimits(:,2))]);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%

    clear ylimits
    ylimits = [];
    % IP tip deflection
    if contains(config_name,'FIP')
        nexttile(3)
        hold on
        plot(t,tip_def(1,:),'DisplayName',legend_name,'LineStyle',line_style,'Color',color1);
        ax_tip_def_x = gca;
%         title('Tip Deflection')
        % xlabel('Time (s)');
        xticklabels('');
        ylabel('$\delta_{x}$ (mm)');
        ylimits = [ylimits; get(gca,'Ylim')];

        nexttile(6)
        hold on
        plot(t,tip_def(2,:),'DisplayName',legend_name,'LineStyle',line_style,'Color',color1);
        ax_tip_def_y = gca;
        xlabel('Time (s)');
        ylabel('$\delta_{y}$ (mm)');
        ylimits = [ylimits; get(gca,'Ylim')];
        set([ax_tip_def_x,ax_tip_def_y],'Ylim',[min(ylimits(:,1)), max(ylimits(:,2))]);
    end


    %%%%%%%%%%%%%%%%%%%%%%%%%%
    % Save figure
    if save_fig == 1
        h = get(gca,'Children');
        lg = legend(gca,h(:));
        lg.Orientation = 'horizontal';
        lg.Layout.Tile = 'South'; % <-- Legend placement with tiled layout
        set(gca,'Children',h(end:-1:1))
        lg = legend(gca,h(:));
        set(gca,'Children',h(1:end))
%         fig = gcf;
%         fig.Position(3) = fig.Position(3) + 1000;
%         disp('Please adjust location of legend, then press enter...')
%         pause
        savefig(['.\results\co-plot\version',num2str(version),'\FeaturesPlot'])
        gcfig = gcf;
        print2eps(gcfig,['.\results\co-plot\version',num2str(version),'\fig_feats.eps'],16,9,'painters');
        print2png(gcfig,['.\results\co-plot\version',num2str(version),'\fig_feats.png'],16,9);
        clear gcfig
    end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% top_fig
top_fig = gcf;
figure(top_fig);
