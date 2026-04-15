function u_cmd = MPCController_stepNew(in)


persistent N nx nu N_MAX initialized
persistent step_count t_sum t_min_val t_max_val t_history display_rate dt_ms
persistent all_step_ms all_solver_ms all_overhead_ms all_iters
persistent all_status all_lqr all_cabin_err all_success
persistent Tamb_C_log prev_in_flag mpc_timing_cache


    %% ── One-time init ────────────────────────────────────────────────────
  if isempty(initialized) || ~initialized || step_count >= 200000 - 10
    N     = double(evalin('base','N'));
    nx    = double(evalin('base','nx'));
    nu    = double(evalin('base','nu'));
    N_MAX = double(evalin('base','N_MAX'));
    initialized = true;

    step_count      = 0;
    t_sum           = 0;
    t_min_val       = inf;
    t_max_val       = 0;
    t_history       = zeros(1,100);
    display_rate    = 10;
    dt_ms           = double(evalin('base','Ts')) * 1000;

    max_steps       = 200000;   % raised from 10000 — handles longer runs
    all_step_ms     = nan(max_steps,1);
    all_solver_ms   = nan(max_steps,1);
    all_overhead_ms = nan(max_steps,1);
    all_iters       = nan(max_steps,1);
    all_status      = cell(max_steps,1);
    all_lqr         = nan(max_steps,1);
    all_cabin_err   = nan(max_steps,1);
    all_success     = false(max_steps,1);
    Tamb_C_log      = nan;
    prev_in_flag    = 1;
    mpc_timing_cache = [];

    fprintf('[MPCController_stepNew] Initialised (step_count reset to 0)\n');
 end

    %% ── Unpack input vector ──────────────────────────────────────────────
    idx    = 1;
    x_meas = in(idx:idx+nx-1); idx = idx+nx;

    states_measured = [x_meas(1)+273.15;   % 1 motT   [degC->K]
                       x_meas(2)+273.15;   % 2 invT
                       x_meas(3)+273.15;   % 3 dcdcT
                       x_meas(4);          % 4 SOC [-]
                       x_meas(5)+273.15;   % 5 batT
                       x_meas(6)*1e6;      % 6 p_in  [MPa->Pa]
                       x_meas(7)*1e6;      % 7 p_out [MPa->Pa]
                       x_meas(8)+273.15;   % 8 cabIntT
                       x_meas(9)+273.15];  % 9 cabAirT

    speed      = in(idx:idx+N_MAX-1); idx = idx+N_MAX;
    Tamb_C_vec = in(idx:idx+N_MAX-1); idx = idx+N_MAX;
    speed      = speed(1:N);
    Tamb_C_vec = Tamb_C_vec(1:N);

    motQ_vec     = in(idx:idx+N-1); idx = idx+N;
    invQ_vec     = in(idx:idx+N-1); idx = idx+N;
    dcdcQ_vec    = in(idx:idx+N-1); idx = idx+N;
    batImeas_vec = in(idx:idx+N-1); idx = idx+N;

    Tref_K             = in(idx)+273.15; idx = idx+1;
    clntInvTout_K_meas = in(idx)+273.15; idx = idx+1;
    clntBatTin_K_meas  = in(idx)+273.15;

    if isnan(Tamb_C_log)
        Tamb_C_log = Tamb_C_vec(1);
    end
    dt_ms = double(evalin('base','Ts')) * 1000;

    %% ── Timed MPC solve ──────────────────────────────────────────────────
    t_start = tic;
  

    [u_cmd, info] = MPCController_step_coreLQRNew( ...
        states_measured, Tamb_C_vec(1), Tref_K, ...
        clntInvTout_K_meas, clntBatTin_K_meas, ...
        dcdcQ_vec, motQ_vec, invQ_vec, ...
        speed, batImeas_vec);

    t_elapsed_ms = toc(t_start) * 1000;
    info.lqrFallback = 0;

    %% ── Accumulate per-step data ─────────────────────────────────────────
    step_count = step_count + 1;
    t_sum      = t_sum + t_elapsed_ms;
    t_min_val  = min(t_min_val, t_elapsed_ms);
    t_max_val  = max(t_max_val, t_elapsed_ms);

    buf_idx            = mod(step_count-1,100)+1;
    t_history(buf_idx) = t_elapsed_ms;

    if step_count <= numel(all_step_ms)
        all_step_ms(step_count)     = t_elapsed_ms;
        all_solver_ms(step_count)   = info.t_solver_ms;
        all_overhead_ms(step_count) = t_elapsed_ms - info.t_solver_ms;
        all_iters(step_count)       = double(info.iter);
        all_status{step_count}      = char(info.status);
        all_lqr(step_count)         = info.lqrFallback;
        all_cabin_err(step_count)   = states_measured(9) - Tref_K;
        all_success(step_count)     = info.success;
    end

    %% ── Build mpc_timing and push to workspace every step ───────────────
    n = step_count;
    vm  = all_step_ms(1:n);
    vs  = all_solver_ms(1:n);
    vo  = all_overhead_ms(1:n);
    vi  = all_iters(1:n);
    vi_valid = vi(~isnan(vi));

    s.n_steps           = n;
    s.mean_total_ms     = mean(vm);
    s.median_total_ms   = median(vm);
    s.max_total_ms      = max(vm);
    s.min_total_ms      = min(vm);
    s.p95_total_ms      = prctile(vm,95);
    s.p99_total_ms      = prctile(vm,99);
    s.std_total_ms      = std(vm);
    s.mean_solver_ms    = mean(vs);
    s.max_solver_ms     = max(vs);
    s.p95_solver_ms     = prctile(vs,95);
    s.mean_overhead_ms  = mean(vo);
    s.max_overhead_ms   = max(vo);
    s.mean_iters        = mean(vi_valid);
    s.max_iters         = max(vi_valid);
    s.p95_iters         = isempty(vi_valid)*0 + ~isempty(vi_valid)*prctile(vi_valid,95);
    s.n_failed          = sum(~all_success(1:n));
    s.n_lqr_discounted  = sum(all_lqr(1:n)==1);
    s.n_lqr_identity    = sum(all_lqr(1:n)==2);
    s.n_deadline_miss   = sum(vm > dt_ms);
    s.n_above_80pct     = sum(vm > 0.8*dt_ms);
    s.deadline_miss_pct = 100*s.n_deadline_miss/n;

    mpc_timing.step_times_ms   = vm;
    mpc_timing.solver_times_ms = vs;
    mpc_timing.overhead_ms     = vo;
    mpc_timing.iter_counts     = vi;
    mpc_timing.solve_status    = all_status(1:n);
    mpc_timing.lqr_fallback    = all_lqr(1:n);
    mpc_timing.cabin_error_K   = all_cabin_err(1:n);
    mpc_timing.success_flags   = all_success(1:n);
    mpc_timing.Tamb_C          = Tamb_C_log;
    mpc_timing.Ts_s            = dt_ms/1000;
    mpc_timing.N               = N;
    mpc_timing.summary         = s;

    assignin('base','mpc_timing',mpc_timing);
    mpc_timing_cache = mpc_timing;   % keep local copy for end-of-run save

    %% ── Periodic display ─────────────────────────────────────────────────
    t_mean   = t_sum/step_count;
    t_recent = mean(t_history(1:min(step_count,100)));

    if mod(step_count,display_rate) == 0
        if info.success
            status_str = 'OK';
        else
            status_str = sprintf('FAILED(%s)',char(info.status));
        end
        switch info.lqrFallback
            case 0,    lqr_str='DARE';
            case 1,    lqr_str='DARE-discounted';
            case 2,    lqr_str='identity(!)';
            otherwise, lqr_str='unknown';
        end
        iter_disp = double(info.iter);
        if isnan(iter_disp), iter_disp=-1; end

        fprintf('\n╔══════════════════════════════════════════════════╗\n');
        fprintf('║        MPC Timing Report  (step %5d)           ║\n',step_count);
        fprintf('╠══════════════════════════════════════════════════╣\n');
        fprintf('║  This step   : %7.1f ms / %.0f ms  (%5.1f%%)   \n', ...
                t_elapsed_ms,dt_ms,100*t_elapsed_ms/dt_ms);
        fprintf('║  Status      : %-20s  iters: %3d      \n',status_str,iter_disp);
        % fprintf('║  Recent mean : %7.1f ms  (last 100 steps)       \n',t_recent);
        % fprintf('║  All-time    : mean %6.1f  min %5.1f  max %6.1f ms\n', ...
                % t_mean,t_min_val,t_max_val);
        fprintf('╠══════════════════════════════════════════════════╣\n');
        fprintf('║  Cabin air T : %6.2f K  (ref: %6.2f K)  err: %+.2f K\n', ...
                states_measured(9),Tref_K,states_measured(9)-Tref_K);
        fprintf('║  Tamb        : %6.2f degC                        \n',Tamb_C_vec(1));
        fprintf('║  LQR terminal: %-20s                  \n',lqr_str);
        % fprintf('║  Deadline >100%%: %d/%d steps (%.1f%%)\n', ...
                % s.n_deadline_miss,n,s.deadline_miss_pct);
        fprintf('╚══════════════════════════════════════════════════╝\n');
    end

    %% ── Per-step timing line and deadline warnings ───────────────────────
    fprintf('║  This step   : %7.1f ms / %.0f ms  (%5.1f%%)   \n', ...
            t_elapsed_ms,dt_ms,100*t_elapsed_ms/dt_ms);
    fprintf('║  Solver only : %7.1f ms  (%5.1f%% of step)      \n', ...
            info.t_solver_ms,100*info.t_solver_ms/dt_ms);

    iter_count = double(info.iter);
    if isnan(iter_count), iter_count=-1; end

    if t_elapsed_ms > 0.95*dt_ms
        warning('MPCController_step:DeadlineMiss', ...
            'Step %d: %.1f ms — NEAR DEADLINE (%.0f ms budget). Status: %s, iters: %d', ...
            step_count,t_elapsed_ms,dt_ms,char(info.status),iter_count);
    elseif t_elapsed_ms > 0.8*dt_ms
        warning('MPCController_step:SlowSolve', ...
            'Step %d: %.1f ms — exceeds 80%% budget (%.0f ms). Status: %s, iters: %d', ...
            step_count,t_elapsed_ms,dt_ms,char(info.status),iter_count);
    end

    if ~info.success
        warning('MPCController_step:Infeasible', ...
            'Step %d: INFEASIBLE (%s). CabAirT=%.1fK Tref=%.1fK Tamb=%.1fC p_in=%.2fbar p_out=%.2fbar SOC=%.3f', ...
            step_count,char(info.status), ...
            states_measured(9),Tref_K,Tamb_C_vec(1), ...
            states_measured(6)/1e5,states_measured(7)/1e5,states_measured(4));
    end

    prev_in_flag = 1;

end



%% =========================================================================
function save_mpc_timing_to_disk(mpc_timing)


    if nargin<1 || isempty(mpc_timing)
        if evalin('base','exist(''mpc_timing'',''var'')')
            mpc_timing = evalin('base','mpc_timing');
        else
            warning('save_mpc_timing_to_disk:noData', ...
                    'mpc_timing not found in base workspace.');
            return
        end
    end

    Tamb_C = mpc_timing.Tamb_C;
    if isnan(Tamb_C), tag='unknownTemp';
    else, tag=sprintf('%ddegC',round(Tamb_C)); end

    out_dir = fullfile(pwd,'simulation_results');
    if ~exist(out_dir,'dir'), mkdir(out_dir); end

    fname = fullfile(out_dir,sprintf('mpc_timing_%s.mat',tag));
    save(fname,'mpc_timing');
    fprintf('[MPC Timing] Saved: %s\n',fname);

    % print_mpc_timing_report(mpc_timing);
end


%% =========================================================================
function print_mpc_timing_report(mpc_timing)
% Prints formatted timing report for the paper.

    s  = mpc_timing.summary;
    Ts = mpc_timing.Ts_s * 1000;
    T  = mpc_timing.Tamb_C;

    fprintf('\n╔══════════════════════════════════════════════════════════════╗\n');
    fprintf('║  MPC TIMING REPORT  —  Tamb = %.0f C                          \n',T);
    fprintf('╠══════════════════════════════════════════════════════════════╣\n');
    fprintf('║  Sample time budget : %.0f ms   N = %d steps   Steps: %d       \n', ...
            Ts,mpc_timing.N,s.n_steps);
    fprintf('╠══════════════════════════════════════════════════════════════╣\n');
    fprintf('║  WALL-CLOCK TIME PER STEP                                    \n');
    fprintf('║    Mean             : %7.1f ms                               \n',s.mean_total_ms);
    fprintf('║    Median           : %7.1f ms                               \n',s.median_total_ms);
    fprintf('║    Std dev          : %7.1f ms                               \n',s.std_total_ms);
    fprintf('║    Min              : %7.1f ms                               \n',s.min_total_ms);
    fprintf('║    Worst case (Max) : %7.1f ms                               \n',s.max_total_ms);
    fprintf('║    95th percentile  : %7.1f ms                               \n',s.p95_total_ms);
    fprintf('║    99th percentile  : %7.1f ms                               \n',s.p99_total_ms);
    fprintf('╠══════════════════════════════════════════════════════════════╣\n');
    fprintf('║  IPOPT SOLVER TIME                                           \n');
    fprintf('║    Mean             : %7.1f ms                               \n',s.mean_solver_ms);
    fprintf('║    Max              : %7.1f ms                               \n',s.max_solver_ms);
    fprintf('║    95th percentile  : %7.1f ms                               \n',s.p95_solver_ms);
    fprintf('╠══════════════════════════════════════════════════════════════╣\n');
    fprintf('║  OVERHEAD (CoolProp + terminal NLP)                          \n');
    fprintf('║    Mean             : %7.1f ms                               \n',s.mean_overhead_ms);
    fprintf('║    Max              : %7.1f ms                               \n',s.max_overhead_ms);
    fprintf('╠══════════════════════════════════════════════════════════════╣\n');
    fprintf('║  IPOPT ITERATIONS                                            \n');
    fprintf('║    Mean             : %7.1f iters                            \n',s.mean_iters);
    fprintf('║    Max              : %7.0f iters                            \n',s.max_iters);
    fprintf('║    95th percentile  : %7.1f iters                            \n',s.p95_iters);
    fprintf('╠══════════════════════════════════════════════════════════════╣\n');
    fprintf('║  RELIABILITY                                                 \n');
    fprintf('║    Failed solves    : %4d / %d  (%.1f%%)                      \n', ...
            s.n_failed,s.n_steps,100*s.n_failed/max(s.n_steps,1));
    fprintf('║    Deadline misses  : %4d / %d  (%.1f%%)  [> %.0f ms]         \n', ...
            s.n_deadline_miss,s.n_steps,s.deadline_miss_pct,Ts);
    fprintf('║    Above 80%% budget : %4d / %d  (%.1f%%)                      \n', ...
            s.n_above_80pct,s.n_steps,100*s.n_above_80pct/max(s.n_steps,1));
    fprintf('╠══════════════════════════════════════════════════════════════╣\n');
    fprintf('║  TERMINAL COST FALLBACKS                                     \n');
    fprintf('║    DARE (nominal)   : %d steps                                \n', ...
            s.n_steps - s.n_lqr_discounted - s.n_lqr_identity);
    fprintf('║    Discounted DARE  : %d steps                                \n',s.n_lqr_discounted);
    fprintf('║    Identity (worst) : %d steps                                \n',s.n_lqr_identity);
    fprintf('╚══════════════════════════════════════════════════════════════╝\n');

    fprintf('\n  Paper summary (copy to Section VII):\n');
    fprintf('  "The NMPC solve (IPOPT/MA97, N=%d, Ts=%.0f ms) achieved a mean\n', ...
            mpc_timing.N,Ts);
    fprintf('  solver time of %.1f ms (std %.1f ms, worst-case %.1f ms,\n', ...
            s.mean_solver_ms,s.std_total_ms,s.max_solver_ms);
    fprintf('  95th percentile %.1f ms) against a %.0f ms sample-time budget.\n', ...
            s.p95_solver_ms,Ts);
    fprintf('  Deadline violations (> %.0f ms) occurred in %.1f%% of steps."\n\n', ...
            Ts,s.deadline_miss_pct);
end