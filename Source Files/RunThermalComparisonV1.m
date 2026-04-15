function RunThermalComparisonV1(varargin)
% RunThermalComparison — see inline comments for usage
%
% Usage:
%   RunThermalComparison()                      % interactive menu
%   RunThermalComparison('both', [-5,-7,-10])   % all combos
%   RunThermalComparison('nmpc', -10)           % single run
%   RunThermalComparison('rule', [-5,-10])
%
% After each run the script:
%   1. Saves workspace to simulation_results/<timestamp>/<ctrl>_<T>degC.mat
%   2. Saves a 3-panel temperature figure (cabin / battery / motor) as PNG
% After all runs:
%   3. Saves per-temperature Rule vs NMPC overlay plots
%   4. Saves a combined 3-column overview figure
%   5. Saves comparison_results to base workspace and to .mat

  MODEL = 'ElectricVehicleThermalManagementWithHeatPump';

    if nargin == 0
        [ctrl_sel, temp_sel] = interactive_menu();
    elseif nargin == 2
        ctrl_sel = varargin{1};
        temp_sel = varargin{2};
    else
        error('Usage: RunThermalComparison() or RunThermalComparison(ctrl,temps)');
    end

    controllers = strcmp(ctrl_sel,'both') * 1;   % dummy — build list below
    if strcmp(ctrl_sel,'both')
        controllers = {'rule','nmpc'};
    else
        controllers = {ctrl_sel};
    end
    temps = temp_sel(:)';

    for T = temps
        if ~ismember(T,[-5,-7,-10])
            error('Temperature %d not supported. Choose -5, -7, or -10.',T);
        end
    end

    if ~bdIsLoaded(MODEL), load_system(MODEL); end

    out_dir = fullfile(pwd,'simulation_results',datestr(now,'yyyy-mm-dd_HH-MM-SS')); %#ok<TNOW1,DATST>
    if ~exist(out_dir,'dir'), mkdir(out_dir); end
    fprintf('[Run] Output directory: %s\n', out_dir);

    results  = struct();
    all_data = struct();
    n_total  = numel(controllers)*numel(temps);
    n_done   = 0;

    for c = 1:numel(controllers)
        ctrl = controllers{c};
        for T = temps
            n_done     = n_done+1;
            field_temp = sprintf('T%d',abs(T));

            fprintf('\n==== Run %d/%d  ctrl=%s  Tamb=%d C ====\n', ...
                    n_done,n_total,upper(ctrl),T);

            set_cold_weather_scenario(MODEL,T);
            switch_controller(MODEL,ctrl);

            if strcmp(ctrl,'nmpc')
                for cf = {fullfile(pwd,'nmpc_warm_cache_v3.mat'), ...
                          fullfile(pwd,'nmpc_warm_cache.mat')}
                    if isfile(cf{1}), delete(cf{1}); end
                end
                clear MPCController_step_coreLQRNew_v3
                clear MPCController_step_coreLQRNew
                clear MPCController_stepNew
            end

            t0 = tic;
            simOut = sim(MODEL);
            fprintf('[Sim] Done in %.1f s\n', toc(t0));

            simlog_varname = ['simlog_' strrep(MODEL,' ','_')];
            % if isprop(simOut,simlog_varname)
            %     simlog = simOut.(simlog_varname);
            % elseif exist('simlog_ElectricVehicleThermalManagementWithHeatPump', 'var')
            %     simlog = evalin('base',simlog_varname);
            % else
            %     error('simlog not found. Check model logging settings.');
            % end

            stats    = compute_energy_results(simlog_ElectricVehicleThermalManagementWithHeatPump, MODEL);
            ts       = extract_temperatures(simlog_ElectricVehicleThermalManagementWithHeatPump, simOut);
            stats.ts = ts;

            if ~isfield(results,ctrl), results.(ctrl) = struct(); end
            results.(ctrl).(field_temp)     = stats;
            all_data.(ctrl).(field_temp).ts = ts;
            all_data.(ctrl).(field_temp).T  = T;

            save_run_workspace(out_dir, ctrl, T, simOut, simlog_ElectricVehicleThermalManagementWithHeatPump, stats);

            fig = plot_single_run(ts, ctrl, T, stats);
            save_figure(fig, out_dir, sprintf('temps_%s_%ddegC', ctrl, abs(T)));
            close(fig);
        end
    end

    if numel(controllers) == 2
        for T = temps
            ft = sprintf('T%d',abs(T));
            if isfield(results,'rule') && isfield(results.rule,ft) && ...
               isfield(results,'nmpc') && isfield(results.nmpc,ft)
                print_comparison(results.rule.(ft), results.nmpc.(ft), T);
                fig = plot_comparison_single_temp( ...
                           all_data.rule.(ft).ts, all_data.nmpc.(ft).ts, T);
                save_figure(fig, out_dir, sprintf('comparison_%ddegC', abs(T)));
                close(fig);
            end
        end
        fig = plot_all_temps_comparison(all_data, temps);
        
        save_figure(fig, out_dir, 'comparison_all_temps');
        close(fig);
        print_summary_table(results, temps);
        fig  = plot_energy_accumulation(results.rule.(ft), results.nmpc.(ft), T);
        save_figure(fig, out_dir, sprintf('energy_%ddegC', abs(T)));
        close(fig);
    end

    assignin('base','comparison_results',results);
    master_file = fullfile(out_dir,'comparison_results.mat');
    save(master_file,'results');
    fprintf('\n[Done] Results: %s\n', master_file);
end

% -------------------------------------------------------------------------
function ts = extract_temperatures(simlog, simOut)
% UPDATE simlog paths to match your Simscape hierarchy.
    ts = struct();
    try
        ts.t = simlog.Compressor.Compressor.mechanical_power.series.time;
    catch
        ts.t = simOut.tout;
    end
    % Cabin air
    try
        ts.cabAirT_C = simlog.Cabin.Cabin_Air_Volume.T_I.series.values('degC');
    catch
        ts.cabAirT_C = NaN(size(ts.t));
        warning('Cabin air T path not found — update extract_temperatures().');
    end
    % Battery
    try
        ts.batT_C = 1/4*( simlog.Battery.Pack_1.Thermal_Model.T.series.values('degC')...
            +simlog.Battery.Pack_2.Thermal_Model.T.series.values('degC')...
            +simlog.Battery.Pack_3.Thermal_Model.T.series.values('degC')...
            +simlog.Battery.Pack_4.Thermal_Model.T.series.values('degC'));
    catch
        ts.batT_C = NaN(size(ts.t));
        warning('Battery T path not found — update extract_temperatures().');
    end
    % Motor
    try
        ts.motT_C = simlog.Motor.Motor.T.series.values('degC');
    catch
        ts.motT_C = NaN(size(ts.t));
        warning('Motor T path not found — update extract_temperatures().');
    end
    % MPC logged state (optional)
    try
        Ts = evalin('base','Ts');
        cl = evalin('base','cabAirT_log');
        tm = (0:numel(cl)-1)'*Ts;
        ts.cabAirT_mpc_C = interp1(tm, double(cl)-273.15, ts.t,'linear',NaN);
    catch
        ts.cabAirT_mpc_C = [];
    end
    % Setpoint
    try
        ts.Tref_C = evalin('base','Tref_K')-273.15;
    catch
        ts.Tref_C = 21;
    end
end

% -------------------------------------------------------------------------
function fig = plot_single_run(ts, ctrl, T_amb, stats)
    t_min = ts.t/60;
    fig   = figure('Name',sprintf('%s @ %d C',upper(ctrl),T_amb), ...
                   'Position',[100 100 1000 750],'Color','white');

    % Cabin
    ax1 = subplot(3,1,1); hold(ax1,'on');
    plot(ax1, t_min, ts.cabAirT_C, 'b-','LineWidth',1.8,'DisplayName','Cabin Air T');
    if ~isempty(ts.cabAirT_mpc_C) && ~all(isnan(ts.cabAirT_mpc_C))
        plot(ax1, t_min, ts.cabAirT_mpc_C,'b--','LineWidth',1.0, ...
             'DisplayName','Cabin Air T (MPC state)');
    end
    yline(ax1, ts.Tref_C,'r--','LineWidth',1.5,'DisplayName', ...
          sprintf('Setpoint %.0f C',ts.Tref_C));
    yline(ax1, T_amb,'k:','LineWidth',1.0,'DisplayName', ...
          sprintf('T_{amb} %d C',T_amb));
    if ~isnan(stats.t_settle_s)
        xline(ax1,stats.t_settle_s/60,'g-','LineWidth',1.2, ...
              'DisplayName',sprintf('Settled %.1f min',stats.t_settle_s/60));
    end
    ylabel(ax1,'Temp [C]');
    title(ax1,sprintf('%s | Tamb=%d C | Cabin Air',upper(ctrl),T_amb));
    legend(ax1,'Location','southeast','FontSize',8);
    grid(ax1,'on'); xlim(ax1,[0 t_min(end)]);

    % Battery
    ax2 = subplot(3,1,2); hold(ax2,'on');
    plot(ax2, t_min, ts.batT_C,'r-','LineWidth',1.8,'DisplayName','Battery T');
    yline(ax2, T_amb,'k:','LineWidth',1.0,'DisplayName','T_{amb}');
    yline(ax2, 15,'g--','LineWidth',1.0,'DisplayName','Pref lo 15 C');
    yline(ax2, 50,'m--','LineWidth',1.0,'DisplayName','Pref hi 50 C');
    ylabel(ax2,'Temp [C]'); title(ax2,'Battery Temperature');
    legend(ax2,'Location','southeast','FontSize',8);
    grid(ax2,'on'); xlim(ax2,[0 t_min(end)]);

    % Motor
    ax3 = subplot(3,1,3); hold(ax3,'on');
    plot(ax3, t_min, ts.motT_C,'m-','LineWidth',1.8,'DisplayName','Motor T');
    yline(ax3, T_amb,'k:','LineWidth',1.0,'DisplayName','T_{amb}');
    yline(ax3, 75,'r--','LineWidth',1.0,'DisplayName','Pref hi 75 C');
    xlabel(ax3,'Time [min]'); ylabel(ax3,'Temp [C]');
    title(ax3,'Motor Temperature');
    legend(ax3,'Location','northeast','FontSize',8);
    grid(ax3,'on'); xlim(ax3,[0 t_min(end)]);

    annotation(fig,'textbox',[0.72 0.01 0.26 0.06], ...
               'String',sprintf('Total: %.1f Wh | Settle: %.1f min', ...
                                stats.E_total_Wh, stats.t_settle_s/60), ...
               'EdgeColor','none','FontSize',8,'Color',[0.3 0.3 0.3]);
end

% -------------------------------------------------------------------------
function fig = plot_comparison_single_temp(ts_rule, ts_nmpc, T_amb)
    fig = figure('Name',sprintf('Rule vs NMPC @ %d C',T_amb), ...
                 'Position',[120 120 1100 800],'Color','white');
    t_r    = ts_rule.t/60;  t_n = ts_nmpc.t/60;
    tmax   = max(t_r(end), t_n(end));
    Tref_C = ts_rule.Tref_C;

    ts_list = {ts_rule, ts_nmpc};
    t_list  = {t_r,     t_n};
    labels  = {'Rule-Based', 'NMPC'};
    cols    = {[0.8 0.2 0.2], [0.1 0.4 0.8]};
    lw      = [1.5, 2.0];
    fields  = {'cabAirT_C','batT_C','motT_C'};
    ylabels = {'Cabin Air T [C]','Battery T [C]','Motor T [C]'};
    ttitles = {'Cabin Air Temperature','Battery Temperature','Motor Temperature'};

    for p = 1:3
        ax = subplot(3,1,p); hold(ax,'on');
        for ci = 1:2
            y = ts_list{ci}.(fields{p});
            if ~all(isnan(y))
                plot(ax,t_list{ci},y,'-','Color',cols{ci},'LineWidth',lw(ci), ...
                     'DisplayName',labels{ci});
            end
        end
        yline(ax,T_amb,'k:','LineWidth',1.0,'DisplayName',sprintf('Tamb %d C',T_amb));
        if p==1
            yline(ax,Tref_C,'k--','LineWidth',1.5, ...
                  'DisplayName',sprintf('Setpoint %.0f C',Tref_C));
        end
        ylabel(ax,ylabels{p});
        title(ax,sprintf('%s | Tamb = %d C',ttitles{p},T_amb));
        legend(ax,'Location','southeast','FontSize',9);
        grid(ax,'on'); xlim(ax,[0 tmax]);
    end
    xlabel(ax,'Time [min]');
end

% -------------------------------------------------------------------------
function fig = plot_all_temps_comparison(all_data, temps)
    n = numel(temps);
    fig = figure('Name','All Temperatures Comparison', ...
                 'Position',[50 50 400*n 850],'Color','white');
    fields  = {'cabAirT_C','batT_C','motT_C'};
    ylabels = {'Cabin Air T [C]','Battery T [C]','Motor T [C]'};
    cr = [0.8 0.2 0.2];  cn = [0.1 0.4 0.8];

    for row = 1:3
        for col = 1:n
            T  = temps(col);
            ft = sprintf('T%d',abs(T));
            ax = subplot(3,n,(row-1)*n+col); hold(ax,'on');

            if isfield(all_data,'rule') && isfield(all_data.rule,ft)
                ts = all_data.rule.(ft).ts; y = ts.(fields{row});
                if ~all(isnan(y)), plot(ax,ts.t/60,y,'-','Color',cr,'LineWidth',1.4,'DisplayName','Rule'); end
            end
            if isfield(all_data,'nmpc') && isfield(all_data.nmpc,ft)
                ts = all_data.nmpc.(ft).ts; y = ts.(fields{row});
                if ~all(isnan(y)), plot(ax,ts.t/60,y,'-','Color',cn,'LineWidth',1.8,'DisplayName','NMPC'); end
            end

            yline(ax,T,'k:','LineWidth',0.8);
            if row==1
                try Tref_C=all_data.rule.(ft).ts.Tref_C; catch; Tref_C=21; end
                yline(ax,Tref_C,'k--','LineWidth',1.2);
            end

            grid(ax,'on'); ax.FontSize=8;
            if row==1, title(ax,sprintf('Tamb=%d C',T),'FontSize',9,'FontWeight','bold'); end
            if col==1, ylabel(ax,ylabels{row},'FontSize',8); end
            if row==3, xlabel(ax,'Time [min]','FontSize',8); end
            if row==1 && col==n, legend(ax,'Location','southeast','FontSize',7); end
        end
    end
    sgtitle(fig,'Rule-Based vs NMPC — Cabin / Battery / Motor', ...
            'FontSize',11,'FontWeight','bold');
end

% -------------------------------------------------------------------------
function save_run_workspace(out_dir, ctrl, T_amb, simOut, simlog, stats)
    mat_file = fullfile(out_dir, sprintf('%s_%ddegC.mat',ctrl,abs(T_amb)));
    ws_vars  = struct();
    for vn = {'Ts','N','nx','nu','Tref_K','resampled', ...
              'cabAirT_log','compNrpm_log','heaterPwr_log', ...
              'blower_log','p_out_log','p_in_log'}
        if evalin('base',sprintf('exist(''%s'',''var'')',vn{1}))
            ws_vars.(vn{1}) = evalin('base',vn{1});
        end
    end
    try
        save(mat_file,'simOut','simlog','stats','ws_vars','ctrl','T_amb');
        fprintf('[Save] %s\n', mat_file);
    catch ME
        warning('Full save failed (%s) — saving stats only.%s',E.message);
        save(mat_file,'stats','ws_vars','ctrl','T_amb');
        fprintf('[Save] Stats-only: %s\n', mat_file);
    end
end

% -------------------------------------------------------------------------
% function stats = compute_energy_results(simlog, ~)
%     t  = simlog.Compressor.Compressor.mechanical_power.series.time;
%     Wc = simlog.Compressor.Compressor.mechanical_power.series.values('W');
%     Wm = simlog.Motor_Pump.Motor_Pump.mechanical_power.series.values('W');
%     Wb_p = simlog.Battery_Pump.Battery_Pump.mechanical_power.series.values('W');
%     Wbl  = simlog.Blower.Blower.power.series.values('W');
%     try Wf = simlog.Radiator.Fan.mechanical_power.series.values('W');
%     catch; Wf = zeros(size(t)); warning('Radiator fan path not found.'); end
%     try
%         Ts = evalin('base','Ts'); hp = evalin('base','heaterPwr_log');
%         Wh = interp1((0:numel(hp)-1)'*Ts, double(hp), t,'previous',0);
%     catch; Wh = zeros(size(t)); end
% 
%     We  = Wc/0.81 + Wm/0.88 + Wb_p/0.88 + Wf/(0.85*0.95) + Wbl/0.80 + Wh;
%     h   = 1/3600;
%     stats.E_comp_Wh       = trapz(t,Wc/0.81)*h;
%     stats.E_pump_motor_Wh = trapz(t,Wm/0.88)*h;
%     stats.E_pump_bat_Wh   = trapz(t,Wb_p/0.88)*h;
%     stats.E_fan_Wh        = trapz(t,Wf/(0.85*0.95))*h;
%     stats.E_blower_Wh     = trapz(t,Wbl/0.80)*h;
%     stats.E_heater_Wh     = trapz(t,Wh)*h;
%     stats.E_total_Wh      = trapz(t,We)*h;
%     stats.P_total_mean_W  = mean(We);
%     stats.W_total         = We;
%     stats.t               = t;
%     try
%         cT    = simlog.Cabin.CabinAir.T.series.values('degC');
%         Tr    = evalin('base','Tref_K')-273.15;
%         idx   = find(abs(cT-Tr)<1.0,1,'first');
%         stats.t_settle_s = isempty(idx)*NaN + ~isempty(idx)*t(idx);
%     catch; stats.t_settle_s = NaN; end
%     stats.t_total_min = (t(end)-t(1))/60;
% end

% -------------------------------------------------------------------------
function set_cold_weather_scenario(MODEL, T)
    subsys = [MODEL '/Scenario'];
    mws    = get_param(MODEL,'ModelWorkspace');
    set_param([subsys '/Vehicle Speed [km//hr]'],     'ActiveScenario',        'DriveCycle');
    set_param([subsys '/Variant Source'],             'LabelModeActiveChoice', 'constant');
    set_param([subsys '/Pressure [MPa]'],             'Value', '0.101325');
    set_param([subsys '/Temperature [degC]'],         'Value', num2str(T));
    set_param([subsys '/Relative Humidity'],          'Value', '0.5');
    set_param([subsys '/CO2 Fraction'],               'Value', '4e-4');
    set_param([subsys '/Desired Temperature [degC]'], 'Value', '21');
    set_param([subsys '/Number of Occupants'],        'Value', '1');
    pmap = containers.Map([-5,-7,-10],[0.25,0.22,0.20]);
    assignin(mws,'cabin_p_init',0.101325);  assignin(mws,'cabin_T_init',T);
    assignin(mws,'cabin_RH_init',0.5);      assignin(mws,'cabin_CO2_init',4e-4);
    assignin(mws,'coolant_p_init',0.101325);assignin(mws,'coolant_T_init',T);
    assignin(mws,'refrigerant_p_init',pmap(T));
    assignin(mws,'refrigerant_alpha_init',0.60);
    assignin(mws,'battery_T_init',T);       assignin(mws,'battery_Qe_init',0);
    set_param([MODEL '/Scenario'],'Tag',sprintf('cold_%ddegC',T));
    fprintf('[Scenario] Tamb=%d C  ref_p=%.2f MPa\n',T,pmap(T));
end

% -------------------------------------------------------------------------
function switch_controller(MODEL, ctrl)
    vb = [MODEL '/Controls/ControllerSwitch'];
    eb = [MODEL '/Controls/MPCcontrollerEnable'];
    switch ctrl
        case 'rule'
            set_param(vb,'Value','0'); set_param(eb,'Value','0');
            fprintf('[Ctrl] Rule-Based | MPC DISABLED\n');
        case 'nmpc'
            set_param(vb,'Value','1'); set_param(eb,'Value','1');
            fprintf('[Ctrl] NMPC | MPC ENABLED\n');
    end
end

% -------------------------------------------------------------------------
function print_comparison(sr, sn, T)
    lbls = {'Compressor','Motor pump','Bat pump','Fan','Blower','Heater','TOTAL'};
    Er   = [sr.E_comp_Wh;sr.E_pump_motor_Wh;sr.E_pump_bat_Wh; ...
            sr.E_fan_Wh;sr.E_blower_Wh;sr.E_heater_Wh;sr.E_total_Wh];
    En   = [sn.E_comp_Wh;sn.E_pump_motor_Wh;sn.E_pump_bat_Wh; ...
            sn.E_fan_Wh;sn.E_blower_Wh;sn.E_heater_Wh;sn.E_total_Wh];
    fprintf('\n  Energy Comparison  Tamb=%d C\n',T);
    fprintf('  %-12s %10s %10s %10s %8s\n','Component','Rule[Wh]','NMPC[Wh]','Saved[Wh]','Saved[%%]');
    fprintf('  %s\n',repmat('-',1,54));
    for i=1:numel(lbls)
        s=Er(i)-En(i); p=100*s/max(abs(Er(i)),1e-6);
        if i==numel(lbls), fprintf('  %s\n',repmat('-',1,54)); end
        fprintf('  %-12s %10.2f %10.2f %10.2f %7.1f%%\n',lbls{i},Er(i),En(i),s,p);
    end
    fprintf('  Settle: Rule=%.1fs  NMPC=%.1fs\n',sr.t_settle_s,sn.t_settle_s);
end

% -------------------------------------------------------------------------
function print_summary_table(results, temps)
    fprintf('\n  SUMMARY\n');
    fprintf('  %-8s %12s %12s %12s %10s\n','Tamb[C]','Rule[Wh]','NMPC[Wh]','Saved[Wh]','Saved[%%]');
    fprintf('  %s\n',repmat('-',1,58));
    for T=temps
        ft=sprintf('T%d',abs(T));
        if isfield(results,'rule')&&isfield(results.rule,ft)&&...
           isfield(results,'nmpc')&&isfield(results.nmpc,ft)
            Er=results.rule.(ft).E_total_Wh; En=results.nmpc.(ft).E_total_Wh;
            fprintf('  %-8d %12.2f %12.2f %12.2f %9.1f%%\n',T,Er,En,Er-En,100*(Er-En)/max(Er,1e-6));
        end
    end
    fprintf('\n  Settle[s]:  %-8s %12s %12s\n','Tamb[C]','Rule','NMPC');
    fprintf('  %s\n',repmat('-',1,36));
    for T=temps
        ft=sprintf('T%d',abs(T));
        if isfield(results,'rule')&&isfield(results.rule,ft)&&...
           isfield(results,'nmpc')&&isfield(results.nmpc,ft)
            fprintf('  %-8d %12.1f %12.1f\n',T,...
                    results.rule.(ft).t_settle_s,results.nmpc.(ft).t_settle_s);
        end
    end
end

% -------------------------------------------------------------------------
function save_figure(fig, out_dir, base_name)
% Saves fig as:
%   <out_dir>/<base_name>.pdf   — vector PDF (for LaTeX \includegraphics)
%   <out_dir>/<base_name>.tikz  — TikZ/pgfplots source (via matlab2tikz)
%
% Requirements:
%   matlab2tikz  — https://github.com/matlab2tikz/matlab2tikz
%   (add to MATLAB path before calling RunThermalComparison)

    pdf_file  = fullfile(out_dir, [base_name '.pdf']);
    tikz_file = fullfile(out_dir, [base_name '.tikz']);

    %% ── PDF ──────────────────────────────────────────────────────────────
    % ContentType='vector' produces a true vector PDF — no rasterisation.
    % Fonts are embedded so the file renders correctly in any LaTeX compiler.
    try
        exportgraphics(fig, pdf_file, 'ContentType', 'vector');
        fprintf('[Save] PDF:  %s\n', pdf_file);
    catch ME
        warning('save_figure:pdf', 'PDF export failed: %s', ME.message);
    end

    %% ── TikZ ─────────────────────────────────────────────────────────────
    % matlab2tikz converts the current figure to a .tikz file that can be
    % \input{} directly in LaTeX with \usepackage{pgfplots}.
    %
    % Common options used here:
    %   'showInfo'      false  — suppress matlab2tikz console output
    %   'parseStrings'  false  — keep axis labels/titles as-is (no TeX parsing)
    %   'width'         '\linewidth' — figure width adapts to LaTeX column width
    %   'height'        '0.7\linewidth' — aspect ratio ~4:3
    %   'standalone'    false  — output is a tikzpicture, not a full .tex doc
    %
    % If you want a standalone compilable .tex file instead, set 'standalone'
    % to true and change the extension to .tex.
    if exist('matlab2tikz', 'file')
        try
            figure(fig);   % ensure fig is the current figure for matlab2tikz
            matlab2tikz(tikz_file, ...
                'showInfo',     false, ...
                'parseStrings', false, ...
                'width',        '\linewidth', ...
                'height',       '0.65\linewidth', ...
                'standalone',   false);
            fprintf('[Save] TikZ: %s\n', tikz_file);
        catch ME
            warning('save_figure:tikz', 'matlab2tikz failed: %s', ME.message);
        end
    else
        warning('save_figure:noMat2tikz', ...
            ['matlab2tikz not found on MATLAB path.\n' ...
             'Download from https://github.com/matlab2tikz/matlab2tikz\n' ...
             'TikZ file NOT saved for: %s'], base_name);
    end
end

% -------------------------------------------------------------------------
function [ctrl_sel, temp_sel] = interactive_menu()
    fprintf('\n  Thermal Comparison Tool\n\n');
    fprintf('  Controller:  1=Rule  2=NMPC  3=Both\n');
    switch input('  Choice: ')
        case 1, ctrl_sel='rule'; case 2, ctrl_sel='nmpc'; case 3, ctrl_sel='both';
        otherwise, error('Invalid');
    end
    fprintf('  Temperature: 1=-5  2=-7  3=-10  4=All\n');
    switch input('  Choice: ')
        case 1, temp_sel=-5; case 2, temp_sel=-7;
        case 3, temp_sel=-10; case 4, temp_sel=[-5,-7,-10];
        otherwise, error('Invalid');
    end
end

function stats = compute_energy_results(simlog, ~)
    t    = simlog.Compressor.Compressor.mechanical_power.series.time;
    Wc   = simlog.Compressor.Compressor.mechanical_power.series.values('W');
    Wm   = simlog.Motor_Pump.Motor_Pump.mechanical_power.series.values('W');
    Wbp  = simlog.Battery_Pump.Battery_Pump.mechanical_power.series.values('W');
    Wbl  = simlog.Blower.Blower.power.series.values('W');
    try
        Wf = simlog.Radiator.Fan.mechanical_power.series.values('W');
    catch
        Wf = zeros(size(t));
        warning('Radiator fan path not found.');
    end
    try
        Ts = evalin('base','Ts'); hp = evalin('base','heaterPwr_log');
        Wh = interp1((0:numel(hp)-1)'*Ts, double(hp), t,'previous',0);
    catch
        Wh = zeros(size(t));
    end

    % Convert mechanical -> electrical
    Wc_e  = Wc  / 0.81;
    Wm_e  = Wm  / 0.88;
    Wbp_e = Wbp / 0.88;
    Wf_e  = Wf  / (0.85*0.95);
    Wbl_e = Wbl / 0.80;
    Wh_e  = Wh;
    Wtot  = Wc_e + Wm_e + Wbp_e + Wf_e + Wbl_e + Wh_e;

    h = 1/3600;

    % Final energy totals [Wh]
    stats.E_comp_Wh       = trapz(t, Wc_e)  * h;
    stats.E_pump_motor_Wh = trapz(t, Wm_e)  * h;
    stats.E_pump_bat_Wh   = trapz(t, Wbp_e) * h;
    stats.E_fan_Wh        = trapz(t, Wf_e)  * h;
    stats.E_blower_Wh     = trapz(t, Wbl_e) * h;
    stats.E_heater_Wh     = trapz(t, Wh_e)  * h;
    stats.E_total_Wh      = trapz(t, Wtot)  * h;
    stats.P_total_mean_W  = mean(Wtot);

    % Cumulative energy over time [Wh] — for plotting energy accumulation curve
    % cumtrapz gives the running integral at each time step
    stats.E_cumul_comp_Wh  = cumtrapz(t, Wc_e)  * h;
    stats.E_cumul_motor_Wh = cumtrapz(t, Wm_e)  * h;
    stats.E_cumul_bat_Wh   = cumtrapz(t, Wbp_e) * h;
    stats.E_cumul_fan_Wh   = cumtrapz(t, Wf_e)  * h;
    stats.E_cumul_blow_Wh  = cumtrapz(t, Wbl_e) * h;
    stats.E_cumul_heat_Wh  = cumtrapz(t, Wh_e)  * h;
    stats.E_cumul_total_Wh = cumtrapz(t, Wtot)  * h;

    % Save power profiles for later plotting
    stats.t       = t;
    stats.W_comp  = Wc_e;
    stats.W_motor = Wm_e;
    stats.W_bat   = Wbp_e;
    stats.W_fan   = Wf_e;
    stats.W_blow  = Wbl_e;
    stats.W_heat  = Wh_e;
    stats.W_total = Wtot;

    % Time to setpoint
    try
        cT  = simlog.Cabin.CabinAir.T.series.values('degC');
        Tr  = evalin('base','Tref_K') - 273.15;
        idx = find(abs(cT - Tr) < 1.0, 1, 'first');
        stats.t_settle_s = isempty(idx)*NaN + ~isempty(idx)*t(idx);
    catch
        stats.t_settle_s = NaN;
    end

    stats.t_total_min = (t(end) - t(1)) / 60;
end

function fig = plot_energy_accumulation(s_rule, s_nmpc, T_amb)
% Stacked cumulative energy breakdown — Rule vs NMPC

    fig = figure('Name', sprintf('Energy @ %d C', T_amb), ...
                 'Position', [150 150 1100 500], 'Color', 'white');

    t_r = s_rule.t / 60;
    t_n = s_nmpc.t / 60;

    %% Left panel — cumulative total with component stacking
    ax1 = subplot(1,2,1);
    hold(ax1,'on');

    % Rule: stacked area — compressor dominates, then heater, then rest
    E_r = [s_rule.E_cumul_comp_Wh, ...
           s_rule.E_cumul_heat_Wh, ...
           s_rule.E_cumul_blow_Wh, ...
           s_rule.E_cumul_motor_Wh + s_rule.E_cumul_bat_Wh + s_rule.E_cumul_fan_Wh];

    plot(ax1, t_r, s_rule.E_cumul_total_Wh, 'r-',  'LineWidth', 2.0, ...
         'DisplayName', 'Rule-Based total');
    plot(ax1, t_n, s_nmpc.E_cumul_total_Wh, 'b-',  'LineWidth', 2.0, ...
         'DisplayName', 'NMPC total');

    % Shade the saving region
    t_common = linspace(max(t_r(1), t_n(1)), min(t_r(end), t_n(end)), 500);
    E_r_i    = interp1(t_r, s_rule.E_cumul_total_Wh, t_common, 'linear');
    E_n_i    = interp1(t_n, s_nmpc.E_cumul_total_Wh, t_common, 'linear');
    fill(ax1, [t_common, fliplr(t_common)], [E_r_i, fliplr(E_n_i)], ...
         [0.8 0.9 1.0], 'EdgeColor','none','FaceAlpha',0.4, ...
         'DisplayName', 'Energy saved');

    xlabel(ax1,'Time [min]'); ylabel(ax1,'Cumulative Energy [Wh]');
    title(ax1, sprintf('Total Energy — T_{amb} = %d C', T_amb));
    legend(ax1,'Location','northwest','FontSize',9);
    grid(ax1,'on');
    xlim(ax1,[0 max(t_r(end), t_n(end))]);

    % Annotation: total saving
    saving_Wh = s_rule.E_total_Wh - s_nmpc.E_total_Wh;
    saving_pct = 100 * saving_Wh / max(s_rule.E_total_Wh, 1e-6);
    text(ax1, 0.05, 0.92, ...
         sprintf('Saved: %.1f Wh (%.1f%%)', saving_Wh, saving_pct), ...
         'Units','normalized','FontSize',9,'Color',[0 0.5 0], ...
         'FontWeight','bold');

    %% Right panel — per-component bar chart (final values)
    ax2 = subplot(1,2,2);
    lbls      = {'Comp','Heater','Blower','M-Pump','B-Pump','Fan'};
    E_rule_v  = [s_rule.E_comp_Wh; s_rule.E_heater_Wh; s_rule.E_blower_Wh; ...
                 s_rule.E_pump_motor_Wh; s_rule.E_pump_bat_Wh; s_rule.E_fan_Wh];
    E_nmpc_v  = [s_nmpc.E_comp_Wh; s_nmpc.E_heater_Wh; s_nmpc.E_blower_Wh; ...
                 s_nmpc.E_pump_motor_Wh; s_nmpc.E_pump_bat_Wh; s_nmpc.E_fan_Wh];

    b = bar(ax2, [E_rule_v, E_nmpc_v]);
    b(1).FaceColor = [0.8 0.2 0.2];   % Rule — red
    b(2).FaceColor = [0.1 0.4 0.8];   % NMPC — blue
    set(ax2, 'XTickLabel', lbls, 'XTick', 1:numel(lbls));
    ylabel(ax2,'Energy [Wh]');
    title(ax2,'Per-Component Breakdown');
    legend(ax2, {'Rule-Based','NMPC'}, 'Location','northeast','FontSize',9);
    grid(ax2,'on'); ax2.YGrid = 'on'; ax2.XGrid = 'off';
end