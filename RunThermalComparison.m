function RunThermalComparison(varargin)

    %% ── Model name ───────────────────────────────────────────────────────
    MODEL = 'ElectricVehicleThermalManagementWithHeatPump';

    %% ── Ensure matlab2tikz is on path ───────────────────────────────────
    matlab2tikz_path = 'C:\Users\prashant.lokur\OneDrive - Zeekr\Prashant\Phd\01_PaperTask\12_OJVT\ToUpload\matlab2tikz';

    if exist('matlab2tikz', 'file') ~= 2 && exist('matlab2tikz', 'file') ~= 6
        if exist(matlab2tikz_path, 'dir')
            addpath(genpath(matlab2tikz_path));
            rehash;
            fprintf('[Path] Added matlab2tikz path:\n  %s\n', matlab2tikz_path);
        else
            warning('RunThermalComparison:missingMatlab2TikzPath', ...
                'matlab2tikz folder not found:\n%s', matlab2tikz_path);
        end
    end

    fprintf('[Check] matlab2tikz path:\n');
    disp(which('matlab2tikz'));

    if exist('matlab2tikz', 'file') ~= 2 && exist('matlab2tikz', 'file') ~= 6
        warning('RunThermalComparison:noMatlab2Tikz', ...
            'matlab2tikz is still not available on MATLAB path.');
    end

    %% ── Parse inputs ─────────────────────────────────────────────────────
    if nargin == 0
        [ctrl_sel, temp_sel] = interactive_menu();
    elseif nargin == 2
        ctrl_sel = varargin{1};
        temp_sel = varargin{2};
    else
        error('RunThermalComparison:badInput', ...
              'Usage: RunThermalComparisonV2() or RunThermalComparisonV2(controller, temps)');
    end

    ctrl_sel = lower(string(ctrl_sel));

    if ctrl_sel == "both"
        controllers = {'rule', 'nmpc'};
    elseif ctrl_sel == "rule" || ctrl_sel == "nmpc"
        controllers = {char(ctrl_sel)};
    else
        error('RunThermalComparison:badCtrl', ...
              'Controller must be ''rule'', ''nmpc'', or ''both''.');
    end

    temps = temp_sel(:)';   % row vector

    %% ── Validate temperatures ────────────────────────────────────────────
    valid_temps = [-5, -7, -10];
    for T = temps
        if ~ismember(T, valid_temps)
            error('RunThermalComparison:badTemp', ...
                  'Temperature %d not supported. Choose from: -5, -7, -10', T);
        end
    end

   %% ── Output folder with timestamp ────────────────────────────────────────
timestamp = datestr(now, 'yyyy_mm_dd_HH_MM_SS');
outdir = fullfile(pwd, 'Results', ['run_' timestamp]);

if ~exist(outdir, 'dir')
    mkdir(outdir);
end

fprintf('[Output] All files will be saved to:\n  %s\n', outdir);

    %% ── Load model if not loaded ─────────────────────────────────────────
    if ~bdIsLoaded(MODEL)
        fprintf('[RunThermalComparison] Loading model: %s\n', MODEL);
        load_system(MODEL);
    end

    %% ── Run all combinations ─────────────────────────────────────────────
    results = struct();
    n_total = numel(controllers) * numel(temps);
    n_done  = 0;

    for c = 1:numel(controllers)
        ctrl = controllers{c};

        for T = temps
            n_done = n_done + 1;

            fprintf('\n');
            fprintf('════════════════════════════════════════════════════\n');
            fprintf('  Run %d/%d: Controller = %s   Tamb = %d°C\n', ...
                    n_done, n_total, upper(ctrl), T);
            fprintf('════════════════════════════════════════════════════\n');

            % 1) Set scenario parameters
            set_cold_weather_scenario(MODEL, T);

            % 2) Switch controller
            switch_controller(MODEL, ctrl);

            % 3) Clear warm-start cache so each run is independent
              if strcmpi(ctrl, 'nmpc')
                    cache = fullfile(pwd, 'nmpc_warm_cache.mat');
                    if isfile(cache), delete(cache); end
                
                    clear MPCController_step_coreLQRNew_v4
                    clear MPCController_step_coreLQRNew_v3
                    clear MPCController_step_coreLQRNew
                    clear MPCController_step_coreLQRNew_v1
                    clear MPCController_stepNew
                    clear MPCController_stepNewV1    
                    fprintf('[Cache] Cleared MPC persistent state.\n');
               end
            % 4) Run simulation
            fprintf('[Sim] Starting simulation...\n');
            t_sim_start = tic;
            simOut = sim(MODEL);
            t_sim_elapsed = toc(t_sim_start);
            fprintf('[Sim] Completed in %.1f s\n', t_sim_elapsed);
            field_temp = sprintf('T%d', abs(T));
            if strcmpi(ctrl, 'nmpc')
                if evalin('base', 'exist(''mpc_timing'',''var'')')
                    mpc_timing = evalin('base', 'mpc_timing');
            
                    % Save to disk
                    timing_file = fullfile(outdir, ...
                        sprintf('mpc_timing_%s_T%d.mat', ctrl, abs(T)));
                    save(timing_file, 'mpc_timing');
                    fprintf('[Timing] Saved: %s\n', timing_file);
            
                    % Print report inline — avoids dependency on local function
                    s  = mpc_timing.summary;
                    Ts = mpc_timing.Ts_s * 1000;
                   
                    fprintf('  Solver mean   : %.1f ms\n',   s.mean_solver_ms);
                   
            
                    % Store summary in results struct
                    results.(ctrl).(field_temp).mpc_timing = mpc_timing.summary;
                else
                    warning('RunThermalComparison:noTiming', ...
                            'mpc_timing not found in base workspace after NMPC run.');
                end
            end

            % 5) Extract simlog
            simlog = extract_simlog(simOut, MODEL);

            % 6) Compute statistics
            stats = compute_energy_results(simlog);
            stats.controller = ctrl;
            stats.T_amb_C    = T;
            stats.sim_time_s = t_sim_elapsed;
            stats.model_name = MODEL;
            simlog_file = fullfile(outdir, sprintf('simlog_%s_T%d.mat', ctrl, abs(T)));
            try
                save(simlog_file, 'simlog', '-v7.3');   % 
                fprintf('[Save] simlog saved: %s\n', simlog_file);
            catch ME
                warning('RunThermalComparison:simlogSave', ...
                        'Failed to save simlog: %s', ME.message);
            end

            % 7) Store results
            field_temp = sprintf('T%d', abs(T));
            if ~isfield(results, ctrl)
                results.(ctrl) = struct();
            end
            results.(ctrl).(field_temp) = stats;

            % 8) Print single-run summary always
            print_single_run(stats);

            % 9) Save individual plots always
            save_run_plots(stats, ctrl, T, outdir);
        end
    end

    %% ── Save to workspace + MAT file ─────────────────────────────────────
    assignin('base', 'comparison_results', results);
    save(fullfile(outdir, 'comparison_results.mat'), 'results');
    fprintf('\n[Done] Results saved to workspace as ''comparison_results''\n');
    fprintf('[Done] MAT file saved: %s\n', fullfile(outdir, 'comparison_results.mat'));

    %% ── Comparison and summary if both controllers exist ────────────────
    if isfield(results, 'rule') && isfield(results, 'nmpc')
        for T = temps
            field_temp = sprintf('T%d', abs(T));
            if isfield(results.rule, field_temp) && isfield(results.nmpc, field_temp)
                print_comparison(results.rule.(field_temp), ...
                                 results.nmpc.(field_temp), T);

                save_comparison_plots(results.rule.(field_temp), ...
                                      results.nmpc.(field_temp), T, outdir);
            end
        end

        % print_summary_table(results, temps);
        export_summary_table_tex(results, temps, outdir);
    else
        fprintf('\n[Info] Only one controller was run, so pairwise comparison is skipped.\n');
        fprintf('[Info] Individual run results were still printed and saved.\n');
    end

    fprintf('<a href="matlab:print_summary_table(comparison_results, [%s])">Re-print summary table</a>\n', ...
            num2str(temps));
end

%% ═════════════════════════════════════════════════════════════════════════
%% Extract simlog safely
%% ═════════════════════════════════════════════════════════════════════════
function simlog = extract_simlog(simOut, MODEL)

    simlog_varname = ['simlog_' strrep(MODEL, ' ', '_')];

    try
        if isprop(simOut, simlog_varname)
            simlog = simOut.(simlog_varname);
            fprintf('[Sim] simlog extracted from simOut successfully.\n');
            return;
        end
    catch
    end

    try
        if evalin('base', sprintf('exist(''%s'',''var'')', simlog_varname))
            simlog = evalin('base', simlog_varname);
            fprintf('[Sim] simlog found in base workspace.\n');
            return;
        end
    catch
    end

    try
        if evalin('base', 'exist(''simlog'',''var'')')
            simlog = evalin('base', 'simlog');
            fprintf('[Sim] simlog found in base workspace as ''simlog''.\n');
            return;
        end
    catch
    end

    error('RunThermalComparison:noSimlog', ...
          'simlog not found in simOut or base workspace. Check model logging settings.');
end

%% ═════════════════════════════════════════════════════════════════════════
%% Set scenario parameters
%% ═════════════════════════════════════════════════════════════════════════
function set_cold_weather_scenario(MODEL, T_amb_C)

    subsys = [MODEL '/Scenario'];
    mws    = get_param(MODEL, 'ModelWorkspace');

    T_str = num2str(T_amb_C);

    set_param([subsys '/Vehicle Speed [km//hr]'], ...
              'ActiveScenario', 'DriveCycle');

    set_param([subsys '/Variant Source'], ...
              'LabelModeActiveChoice', 'constant');

    set_param([subsys '/Pressure [MPa]'],     'Value', '0.101325');
    set_param([subsys '/Temperature [degC]'], 'Value', T_str);
    set_param([subsys '/Relative Humidity'],  'Value', '0.5');
    set_param([subsys '/CO2 Fraction'],       'Value', '4e-4');

    set_param([subsys '/Desired Temperature [degC]'], 'Value', '21');
    set_param([subsys '/Number of Occupants'],        'Value', '1');

    ref_p_map = containers.Map([-5, -7, -10], [0.25, 0.22, 0.20]);

    assignin(mws, 'cabin_p_init',           0.101325);
    assignin(mws, 'cabin_T_init',           T_amb_C);
    assignin(mws, 'cabin_RH_init',          0.5);
    assignin(mws, 'cabin_CO2_init',         4e-4);
    assignin(mws, 'coolant_p_init',         0.101325);
    assignin(mws, 'coolant_T_init',         T_amb_C);
    assignin(mws, 'refrigerant_p_init',     ref_p_map(T_amb_C));
    assignin(mws, 'refrigerant_alpha_init', 0.60);
    assignin(mws, 'battery_T_init',         T_amb_C);
    assignin(mws, 'battery_Qe_init',        0);

    set_param([MODEL '/Scenario'], 'Tag', ...
              sprintf('cold_weather_%ddegC', T_amb_C));

    fprintf('[Scenario] Tamb = %d°C  |  ref_p_init = %.2f MPa\n', ...
            T_amb_C, ref_p_map(T_amb_C));
end

%% ═════════════════════════════════════════════════════════════════════════
%% Switch controller
%% ═════════════════════════════════════════════════════════════════════════
function switch_controller(MODEL, ctrl)

    variant_block = [MODEL '/Controls/ControllerSwitch'];
    enable_block  = [MODEL '/Controls/MPCcontrollerEnable'];
    blk = [MODEL '/Controls/MPCControllerBlock'];
    get_param(blk,'Handle')     % check that path is valid


    switch lower(ctrl)
        case 'rule'
            set_param(variant_block, 'Value', '0');
            fprintf('[Controller] Switched to: Rule-Based\n');
            fprintf('[Controller] MPC block: DISABLED\n');
            set_param(blk,'Commented','on')
       

        case 'nmpc'
            set_param(variant_block, 'Value', '1');
            fprintf('[Controller] Switched to: NMPC\n');
            fprintf('[Controller] MPC block: ENABLED\n');
            set_param(blk,'Commented','off')

        otherwise
            error('RunThermalComparison:badCtrl', ...
                  'Unknown controller: %s. Use ''rule'' or ''nmpc''.', ctrl);
    end
end

%% ═════════════════════════════════════════════════════════════════════════
%% Energy and temperature computation
%% ═════════════════════════════════════════════════════════════════════════
function stats = compute_energy_results(simlog)

    %% Each signal carries its own time — use it directly for integration
    h = 1/3600;   % J -> Wh

    %% Compressor
    t_comp   = simlog.Compressor.Compressor.mechanical_power.series.time;
    W_comp_m = simlog.Compressor.Compressor.mechanical_power.series.values('W');
    W_comp   = W_comp_m / (0.90*0.90);
    E_comp   = trapz(t_comp, W_comp) * h;

    %% Motor pump
    t_mp   = simlog.Motor_Pump.Motor_Pump.mechanical_power.series.time;
    W_mp_m = simlog.Motor_Pump.Motor_Pump.mechanical_power.series.values('W');
    W_mp   = W_mp_m / 0.88;
    E_mp   = trapz(t_mp, W_mp) * h;

    %% Battery pump
    t_bp   = simlog.Battery_Pump.Battery_Pump.mechanical_power.series.time;
    W_bp_m = simlog.Battery_Pump.Battery_Pump.mechanical_power.series.values('W');
    W_bp   = W_bp_m / 0.88;
    E_bp   = trapz(t_bp, W_bp) * h;

    %% Blower
    t_bl   = simlog.Blower.Blower.power.series.time;
    W_bl_m = simlog.Blower.Blower.power.series.values('W');
    W_bl   = W_bl_m / 0.80;
    E_bl   = trapz(t_bl, W_bl) * h;

    %% Radiator fan
    t_fan   = simlog.Radiator.Fan.mechanical_power.series.time;
    W_fan_m = simlog.Radiator.Fan.mechanical_power.series.values('W');
    W_fan   = W_fan_m / (0.85*0.95);
    E_fan   = trapz(t_fan, W_fan) * h;

    %% Heater
    t_ht  = simlog.Heater.Controlled_Heat_Flow_Rate_Source.Q.series.time;
    W_ht  = simlog.Heater.Controlled_Heat_Flow_Rate_Source.Q.series.values('W');
    E_ht  = trapz(t_ht, W_ht) * h;

    %% Total energy — sum of independently integrated components
    % Do NOT add W signals on a common grid because they have different
    % time vectors. Sum the scalar energy totals instead.
    E_total = E_comp + E_mp + E_bp + E_bl + E_fan + E_ht;

    %% Mean total power — interpolate all signals onto a common fine grid
    % Use the union of all time vectors so no signal is misrepresented
    t_all    = unique([t_comp; t_mp; t_bp; t_bl; t_fan; t_ht]);
    W_comp_i = interp1(t_comp, W_comp, t_all, 'previous', 0);
    W_mp_i   = interp1(t_mp,   W_mp,   t_all, 'previous', 0);
    W_bp_i   = interp1(t_bp,   W_bp,   t_all, 'previous', 0);
    W_bl_i   = interp1(t_bl,   W_bl,   t_all, 'previous', 0);
    W_fan_i  = interp1(t_fan,  W_fan,  t_all, 'previous', 0);
    W_ht_i   = interp1(t_ht,   W_ht,   t_all, 'previous', 0);
    W_total  = W_comp_i + W_mp_i + W_bp_i + W_bl_i + W_fan_i + W_ht_i;

    %% Use the common grid as the canonical time vector for plots
    t = t_all;

    %% Time to setpoint
    try
        t_cab  = simlog.Cabin.Cabin_Air_Volume.T_I.series.time;
        T_cab  = simlog.Cabin.Cabin_Air_Volume.T_I.series.values('degC');
        idx    = find(T_cab >= 20.0, 1, 'first');   % within 1 K of 21 C
        t_settle = isempty(idx)*NaN + ~isempty(idx)*t_cab(idx);
    catch
        t_settle = NaN;
    end

    %% Temperature traces — each on its own time vector
    try
        t_bat_s = simlog.Battery.Pack_1.Thermal_Model.T.series.time;
        T_bat_C = 0.25 * ( ...
            simlog.Battery.Pack_1.Thermal_Model.T.series.values('degC') + ...
            simlog.Battery.Pack_2.Thermal_Model.T.series.values('degC') + ...
            simlog.Battery.Pack_3.Thermal_Model.T.series.values('degC') + ...
            simlog.Battery.Pack_4.Thermal_Model.T.series.values('degC'));
    catch
        t_bat_s = t; T_bat_C = NaN(size(t));
        warning('Battery pack temperature path not found.');
    end

    try
        t_cab_s  = simlog.Cabin.Cabin_Air_Volume.T_I.series.time;
        T_cabAir = simlog.Cabin.Cabin_Air_Volume.T_I.series.values('degC');
    catch
        t_cab_s = t; T_cabAir = NaN(size(t));
        warning('Cabin air temperature path not found.');
    end

    try
        t_mot_s = simlog.Motor.Motor.T.series.time;
        T_mot_C = simlog.Motor.Motor.T.series.values('degC');
    catch
        t_mot_s = t; T_mot_C = NaN(size(t));
        warning('Motor temperature path not found.');
    end

    try
        t_inv_s = simlog.Inverter.Inverter.T.series.time;
        T_inv_C = simlog.Inverter.Inverter.T.series.values('degC');
    catch
        t_inv_s = t; T_inv_C = NaN(size(t));
        warning('Inverter temperature path not found.');
    end

    try
        t_dcdc_s = simlog.DCDC.DCDC.T.series.time;
        T_dcdc_C = simlog.DCDC.DCDC.T.series.values('degC');
    catch
        t_dcdc_s = t; T_dcdc_C = NaN(size(t));
        warning('DCDC temperature path not found.');
    end

    %% Pack stats
    stats.E_comp_Wh       = E_comp;
    stats.E_pump_motor_Wh = E_mp;
    stats.E_pump_bat_Wh   = E_bp;
    stats.E_fan_Wh        = E_fan;
    stats.E_blower_Wh     = E_bl;
    stats.E_heater_Wh     = E_ht;
    stats.E_total_Wh      = E_total;
    stats.P_total_mean_W  = trapz(t_all, W_total) * h * 3600 / ...
                            (t_all(end) - t_all(1));   % mean W over full run

    stats.t_settle_s      = t_settle;
    stats.t_total_min     = (t_all(end) - t_all(1)) / 60;

    % Common interpolated grid for plotting W_total
    stats.t               = t_all;
    stats.W_total         = W_total;

    % Per-component on their own native time vectors
    stats.t_comp          = t_comp;   stats.W_comp       = W_comp;
    stats.t_mp            = t_mp;     stats.W_pump_motor = W_mp;
    stats.t_bp            = t_bp;     stats.W_pump_bat   = W_bp;
    stats.t_bl            = t_bl;     stats.W_blower     = W_bl;
    stats.t_fan           = t_fan;    stats.W_fan        = W_fan;
    stats.t_ht            = t_ht;     stats.W_heater     = W_ht;

    % Temperature traces on their own native time vectors
    stats.t_bat_C         = t_bat_s;  stats.T_bat_C      = T_bat_C;
    stats.t_cabAir_C      = t_cab_s;  stats.T_cabAir_C   = T_cabAir;
    stats.t_mot_C         = t_mot_s;  stats.T_mot_C      = T_mot_C;
    stats.t_inv_C         = t_inv_s;  stats.T_inv_C      = T_inv_C;
    stats.t_dcdc_C        = t_dcdc_s; stats.T_dcdc_C     = T_dcdc_C;
end
%% ═════════════════════════════════════════════════════════════════════════
%% Print single run
%% ═════════════════════════════════════════════════════════════════════════
function print_single_run(s)

    fprintf('\n');
    
end

%% ═════════════════════════════════════════════════════════════════════════
%% Print comparison
%% ═════════════════════════════════════════════════════════════════════════
function print_comparison(s_rule, s_nmpc, T_amb)

    labels = {'Compressor','Motor pump','Battery pump', ...
              'Radiator fan','Cabin blower','Aux heater','TOTAL'};

    E_rule = [s_rule.E_comp_Wh; s_rule.E_pump_motor_Wh; s_rule.E_pump_bat_Wh;
              s_rule.E_fan_Wh;  s_rule.E_blower_Wh;     s_rule.E_heater_Wh;
              s_rule.E_total_Wh];

    E_nmpc = [s_nmpc.E_comp_Wh; s_nmpc.E_pump_motor_Wh; s_nmpc.E_pump_bat_Wh;
              s_nmpc.E_fan_Wh;  s_nmpc.E_blower_Wh;     s_nmpc.E_heater_Wh;
              s_nmpc.E_total_Wh];

    fprintf('\n');
    fprintf('╔══════════════════════════════════════════════════════════════╗\n');
    fprintf('║  Energy Comparison  —  Tamb = %d°C                          \n', T_amb);
    fprintf('╠══════════════════════════════════════════════════════════════╣\n');
    fprintf('║ %-14s  %10s  %10s  %10s  %8s\n', ...
            'Component','Rule [Wh]','NMPC [Wh]','Saved [Wh]','Saved [%]');
    fprintf('║ %s\n', repmat('-',1,56));
    for i = 1:numel(labels)
        saved     = E_rule(i) - E_nmpc(i);
        saved_pct = 100 * saved / max(abs(E_rule(i)), 1e-6);
        if i == numel(labels)
            fprintf('║ %s\n', repmat('-',1,56));
        end
        fprintf('║ %-14s  %10.2f  %10.2f  %10.2f  %7.1f%%\n', ...
                labels{i}, E_rule(i), E_nmpc(i), saved, saved_pct);
    end
    fprintf('╠══════════════════════════════════════════════════════════════╣\n');
    fprintf('║  Time to setpoint:  Rule = %.1f s    NMPC = %.1f s          \n', ...
            s_rule.t_settle_s, s_nmpc.t_settle_s);
    fprintf('╚══════════════════════════════════════════════════════════════╝\n');
end

%% ═════════════════════════════════════════════════════════════════════════
%% Print summary table
%% ═════════════════════════════════════════════════════════════════════════
function print_summary_table(results, temps)

    fprintf('\n');

end

%% ═════════════════════════════════════════════════════════════════════════
%% Save run plots
%% ═════════════════════════════════════════════════════════════════════════
function save_run_plots(stats, ctrl, T, outdir)

    tag = sprintf('%s_T%d', lower(ctrl), abs(T));

    %% ── Helper: resample one (t,y) pair to 1 Hz ─────────────────────────
    rs = @(t,y) resample_1hz(t, y);

    % Resample common power grid once
    [t_pw, W_comp_r]  = rs(stats.t_comp, stats.W_comp);
    [~,    W_mp_r]    = rs(stats.t_mp,   stats.W_pump_motor);
    [~,    W_bp_r]    = rs(stats.t_bp,   stats.W_pump_bat);
    [~,    W_fan_r]   = rs(stats.t_fan,  stats.W_fan);
    [~,    W_bl_r]    = rs(stats.t_bl,   stats.W_blower);
    [~,    W_ht_r]    = rs(stats.t_ht,   stats.W_heater);

    % W_total on its own grid
    [t_tot, W_tot_r]  = rs(stats.t, stats.W_total);

    % Resample temperature signals
    [t_bat_r,  T_bat_r]  = rs(stats.t_bat_C,   stats.T_bat_C);
    [t_cab_r,  T_cab_r]  = rs(stats.t_cabAir_C, stats.T_cabAir_C);
    [t_mot_r,  T_mot_r]  = rs(stats.t_mot_C,   stats.T_mot_C);
    [t_inv_r,  T_inv_r]  = rs(stats.t_inv_C,   stats.T_inv_C);
    [t_dcdc_r, T_dcdc_r] = rs(stats.t_dcdc_C,  stats.T_dcdc_C);

    % =========================
    % Power plot
    % =========================
    fig1 = figure('Visible','off');
    % Use per-component 1Hz grids (they share the same t_pw base grid)
    plot(t_pw, W_comp_r, 'LineWidth',1.2); hold on;
    plot(t_pw, W_mp_r,   'LineWidth',1.2);
    plot(t_pw, W_bp_r,   'LineWidth',1.2);
    plot(t_pw, W_fan_r,  'LineWidth',1.2);
    plot(t_pw, W_bl_r,   'LineWidth',1.2);
    plot(t_pw, W_ht_r,   'LineWidth',1.2);
    plot(t_tot, W_tot_r, 'k','LineWidth',1.8);
    grid on;
    xlabel('Time [s]'); ylabel('Power [W]');
    title(sprintf('%s controller at T_{amb} = %d^\\circC', upper(ctrl), T));
    legend({'Compressor','Motor pump','Battery pump','Fan','Blower','Heater','Total'}, ...
           'Location','best');

    savefig(fig1, fullfile(outdir, [tag '_power.fig']));
    exportgraphics(fig1, fullfile(outdir, [tag '_power.png']), 'Resolution',300);
    try
        matlab2tikz(fullfile(outdir, [tag '_power.tex']), ...
                    'showInfo',false,'standalone',false);
    catch ME
        warning('RunThermalComparison:matlab2tikz', ...
                'matlab2tikz failed for %s power: %s', tag, ME.message);
    end
    close(fig1);

    % =========================
    % Energy bar chart (no time axis — no resampling needed)
    % =========================
    fig2 = figure('Visible','off');
    vals = [stats.E_comp_Wh, stats.E_pump_motor_Wh, stats.E_pump_bat_Wh, ...
            stats.E_fan_Wh,  stats.E_blower_Wh,     stats.E_heater_Wh, ...
            stats.E_total_Wh];
    bar(vals); grid on;
    ylabel('Energy [Wh]');
    title(sprintf('Energy breakdown: %s at %d^\\circC', upper(ctrl), T));
    xticklabels({'Comp','MotPump','BatPump','Fan','Blower','Heater','Total'});

    savefig(fig2, fullfile(outdir, [tag '_energy.fig']));
    exportgraphics(fig2, fullfile(outdir, [tag '_energy.png']), 'Resolution',300);
    try
        matlab2tikz(fullfile(outdir, [tag '_energy.tex']), ...
                    'showInfo',false,'standalone',false);
    catch ME
        warning('RunThermalComparison:matlab2tikz', ...
                'matlab2tikz failed for %s energy: %s', tag, ME.message);
    end
    close(fig2);

    % =========================
    % Battery temperature
    % =========================
    fig_bat = figure('Visible','off');
    plot(t_bat_r, T_bat_r, 'LineWidth',1.5); hold on;
    yline(T,'--','Ambient','LineWidth',1.0);
    grid on; xlabel('Time [s]'); ylabel('Temperature [degC]');
    title(sprintf('Battery temperature: %s at %d^\\circC', upper(ctrl), T));
    legend({'Battery','Ambient'},'Location','best');

    savefig(fig_bat, fullfile(outdir, [tag '_battery_temperature.fig']));
    exportgraphics(fig_bat, fullfile(outdir, [tag '_battery_temperature.png']), 'Resolution',300);
    try
        matlab2tikz(fullfile(outdir, [tag '_battery_temperature.tex']), ...
                    'showInfo',false,'standalone',false);
    catch ME
        warning('RunThermalComparison:matlab2tikz', ...
                'matlab2tikz failed for %s battery: %s', tag, ME.message);
    end
    close(fig_bat);

    % =========================
    % Cabin air temperature
    % =========================
    fig_cab = figure('Visible','off');
    plot(t_cab_r, T_cab_r, 'LineWidth',1.5); hold on;
    yline(T,  '--','Ambient',   'LineWidth',1.0);
    yline(21, '--','Cabin ref', 'LineWidth',1.0);
    grid on; xlabel('Time [s]'); ylabel('Temperature [degC]');
    title(sprintf('Cabin air temperature: %s at %d^\\circC', upper(ctrl), T));
    legend({'Cabin air','Ambient','Cabin ref'},'Location','best');

    savefig(fig_cab, fullfile(outdir, [tag '_cabin_temperature.fig']));
    exportgraphics(fig_cab, fullfile(outdir, [tag '_cabin_temperature.png']), 'Resolution',300);
    try
        matlab2tikz(fullfile(outdir, [tag '_cabin_temperature.tex']), ...
                    'showInfo',false,'standalone',false);
    catch ME
        warning('RunThermalComparison:matlab2tikz', ...
                'matlab2tikz failed for %s cabin: %s', tag, ME.message);
    end
    close(fig_cab);

    % =========================
    % Motor temperature
    % =========================
    fig_mot = figure('Visible','off');
    plot(t_mot_r, T_mot_r, 'LineWidth',1.5); hold on;
    yline(T,'--','Ambient','LineWidth',1.0);
    grid on; xlabel('Time [s]'); ylabel('Temperature [degC]');
    title(sprintf('Motor temperature: %s at %d^\\circC', upper(ctrl), T));
    legend({'Motor','Ambient'},'Location','best');

    savefig(fig_mot, fullfile(outdir, [tag '_motor_temperature.fig']));
    exportgraphics(fig_mot, fullfile(outdir, [tag '_motor_temperature.png']), 'Resolution',300);
    try
        matlab2tikz(fullfile(outdir, [tag '_motor_temperature.tex']), ...
                    'showInfo',false,'standalone',false);
    catch ME
        warning('RunThermalComparison:matlab2tikz', ...
                'matlab2tikz failed for %s motor: %s', tag, ME.message);
    end
    close(fig_mot);

    % =========================
    % Inverter temperature
    % =========================
    fig_inv = figure('Visible','off');
    plot(t_inv_r, T_inv_r, 'LineWidth',1.5); hold on;
    yline(T,'--','Ambient','LineWidth',1.0);
    grid on; xlabel('Time [s]'); ylabel('Temperature [degC]');
    title(sprintf('Inverter temperature: %s at %d^\\circC', upper(ctrl), T));
    legend({'Inverter','Ambient'},'Location','best');

    savefig(fig_inv, fullfile(outdir, [tag '_inverter_temperature.fig']));
    exportgraphics(fig_inv, fullfile(outdir, [tag '_inverter_temperature.png']), 'Resolution',300);
    try
        matlab2tikz(fullfile(outdir, [tag '_inverter_temperature.tex']), ...
                    'showInfo',false,'standalone',false);
    catch ME
        warning('RunThermalComparison:matlab2tikz', ...
                'matlab2tikz failed for %s inverter: %s', tag, ME.message);
    end
    close(fig_inv);

    % =========================
    % DCDC temperature
    % =========================
    fig_dcdc = figure('Visible','off');
    plot(t_dcdc_r, T_dcdc_r, 'LineWidth',1.5); hold on;
    yline(T,'--','Ambient','LineWidth',1.0);
    grid on; xlabel('Time [s]'); ylabel('Temperature [degC]');
    title(sprintf('DCDC temperature: %s at %d^\\circC', upper(ctrl), T));
    legend({'DCDC','Ambient'},'Location','best');

    savefig(fig_dcdc, fullfile(outdir, [tag '_dcdc_temperature.fig']));
    exportgraphics(fig_dcdc, fullfile(outdir, [tag '_dcdc_temperature.png']), 'Resolution',300);
    try
        matlab2tikz(fullfile(outdir, [tag '_dcdc_temperature.tex']), ...
                    'showInfo',false,'standalone',false);
    catch ME
        warning('RunThermalComparison:matlab2tikz', ...
                'matlab2tikz failed for %s DCDC: %s', tag, ME.message);
    end
    close(fig_dcdc);

    fprintf('[Save] Exported run plots for %s at %d C (1 Hz resampled)\n', upper(ctrl), T);
end
%% ═════════════════════════════════════════════════════════════════════════
%% Save comparison plots
%% ═════════════════════════════════════════════════════════════════════════
function save_comparison_plots(s_rule, s_nmpc, T, outdir)

    tag = sprintf('comparison_T%d', abs(T));

    % Energy bar chart — no resampling needed (scalar values)
    E_rule = [s_rule.E_comp_Wh, s_rule.E_pump_motor_Wh, s_rule.E_pump_bat_Wh, ...
              s_rule.E_fan_Wh,  s_rule.E_blower_Wh,     s_rule.E_heater_Wh, ...
              s_rule.E_total_Wh];
    E_nmpc = [s_nmpc.E_comp_Wh, s_nmpc.E_pump_motor_Wh, s_nmpc.E_pump_bat_Wh, ...
              s_nmpc.E_fan_Wh,  s_nmpc.E_blower_Wh,     s_nmpc.E_heater_Wh, ...
              s_nmpc.E_total_Wh];

    fig = figure('Visible','off');
    bar([E_rule(:), E_nmpc(:)]); grid on;
    ylabel('Energy [Wh]');
    title(sprintf('Rule vs NMPC at T_{amb} = %d^\\circC', T));
    xticklabels({'Comp','MotPump','BatPump','Fan','Blower','Heater','Total'});
    legend({'Rule','NMPC'},'Location','best');

    savefig(fig, fullfile(outdir, [tag '_energy.fig']));
    exportgraphics(fig, fullfile(outdir, [tag '_energy.png']), 'Resolution',300);
    try
        matlab2tikz(fullfile(outdir, [tag '_energy.tex']), ...
                    'showInfo',false,'standalone',false);
    catch ME
        warning('RunThermalComparison:matlab2tikz', ...
                'matlab2tikz failed for comparison %dC: %s', T, ME.message);
    end
    close(fig);

    %% ── Temperature comparison plots (Rule vs NMPC, 1 Hz) ────────────────
    temp_signals = { ...
        't_cabAir_C', 'T_cabAir_C', 'Cabin Air Temperature', 'cabin'; ...
        't_bat_C',    'T_bat_C',    'Battery Temperature',   'battery'; ...
        't_mot_C',    'T_mot_C',    'Motor Temperature',     'motor' };

    for k = 1:size(temp_signals,1)
        t_fld  = temp_signals{k,1};
        T_fld  = temp_signals{k,2};
        ttitle = temp_signals{k,3};
        ftag   = temp_signals{k,4};

        [t_r_r, T_r_r] = resample_1hz(s_rule.(t_fld), s_rule.(T_fld));
        [t_n_r, T_n_r] = resample_1hz(s_nmpc.(t_fld), s_nmpc.(T_fld));

        fig_t = figure('Visible','off');
        plot(t_r_r, T_r_r, 'r-',  'LineWidth',1.5,'DisplayName','Rule'); hold on;
        plot(t_n_r, T_n_r, 'b-',  'LineWidth',1.5,'DisplayName','NMPC');
        yline(T, 'k:', 'LineWidth',1.0,'HandleVisibility','off');
        grid on; xlabel('Time [s]'); ylabel('Temperature [degC]');
        title(sprintf('%s: Rule vs NMPC at %d^\\circC', ttitle, T));
        legend('Location','best');

        f_base = fullfile(outdir, sprintf('%s_%s_temperature', tag, ftag));
        savefig(fig_t, [f_base '.fig']);
        exportgraphics(fig_t, [f_base '.png'], 'Resolution',300);
        try
            matlab2tikz([f_base '.tex'], 'showInfo',false,'standalone',false);
        catch ME
            warning('RunThermalComparison:matlab2tikz', ...
                    'matlab2tikz failed for %s %s: %s', tag, ftag, ME.message);
        end
        close(fig_t);
    end

    fprintf('[Save] Exported comparison plots for %d C (1 Hz resampled)\n', T);
end
%% ═════════════════════════════════════════════════════════════════════════
%% Export summary table
%% ═════════════════════════════════════════════════════════════════════════
function export_summary_table_tex(results, temps, outdir)

    fname = fullfile(outdir, 'summary_table.tex');
    fid = fopen(fname, 'w');
    if fid < 0
        warning('RunThermalComparison:fileWrite', ...
                'Could not open summary_table.tex for writing.');
        return;
    end

    fprintf(fid, '%% Auto-generated by RunThermalComparison\n');
    fprintf(fid, '\\begin{tabular}{crrrr}\n');
    fprintf(fid, '\\hline\n');
    fprintf(fid, 'Ambient [$^\\circ$C] & Rule [Wh] & NMPC [Wh] & Saved [Wh] & Saved [\\%%]\\\\\n');
    fprintf(fid, '\\hline\n');

    for T = temps
        field_temp = sprintf('T%d', abs(T));
        if isfield(results, 'rule') && isfield(results, 'nmpc') && ...
           isfield(results.rule, field_temp) && isfield(results.nmpc, field_temp)

            E_rule = results.rule.(field_temp).E_total_Wh;
            E_nmpc = results.nmpc.(field_temp).E_total_Wh;
            saved  = E_rule - E_nmpc;
            pct    = 100 * saved / max(E_rule, 1e-6);

            fprintf(fid, '%d & %.2f & %.2f & %.2f & %.1f\\\\\n', ...
                    T, E_rule, E_nmpc, saved, pct);
        end
    end

    fprintf(fid, '\\hline\n');
    fprintf(fid, '\\end{tabular}\n');
    fclose(fid);

    fprintf('[Save] Exported LaTeX summary table: %s\n', fname);
end

%% ═════════════════════════════════════════════════════════════════════════
%% Interactive menu
%% ═════════════════════════════════════════════════════════════════════════
function [ctrl_sel, temp_sel] = interactive_menu()

    fprintf('\n══════════════════════════════════════\n');
    fprintf('  Thermal Management Comparison Tool  \n');
    fprintf('══════════════════════════════════════\n\n');

    fprintf('Select controller:\n');
    fprintf('  1 — Rule-based only\n');
    fprintf('  2 — NMPC only\n');
    fprintf('  3 — Both (for comparison)\n');
    ctrl_choice = input('Choice [1/2/3]: ');

    switch ctrl_choice
        case 1
            ctrl_sel = 'rule';
        case 2
            ctrl_sel = 'nmpc';
        case 3
            ctrl_sel = 'both';
        otherwise
            error('Invalid choice');
    end

    fprintf('\nSelect ambient temperatures (can choose multiple):\n');
    fprintf('  1 — -5°C\n');
    fprintf('  2 — -7°C\n');
    fprintf('  3 — -10°C\n');
    fprintf('  4 — All three\n');
    temp_choice = input('Choice [1/2/3/4]: ');

    switch temp_choice
        case 1
            temp_sel = -5;
        case 2
            temp_sel = -7;
        case 3
            temp_sel = -10;
        case 4
            temp_sel = [-5, -7, -10];
        otherwise
            error('Invalid choice');
    end

    fprintf('\nRunning: Controller = %s   Temperatures = %s°C\n\n', ...
            upper(ctrl_sel), num2str(temp_sel));
end

function [t_out, y_out] = resample_1hz(t_in, y_in)
    t_out = (ceil(t_in(1)) : floor(t_in(end)))';   
    y_out = interp1(t_in, y_in, t_out, 'linear');
end