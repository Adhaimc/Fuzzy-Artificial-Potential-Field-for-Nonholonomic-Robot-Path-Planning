function fig_stress_test_dense ()
% FIG_STRESS_TEST_DENSE: Dense cluttered environment stress test.
% Tests adaptive law on challenging dense obstacle field where base system
% may struggle. Performs THREE-WAY comparison:
%   1. BASE (no supervisor, no adaptation)
%   2. STATIC+SUP (with supervisor, fixed membership)
%   3. ADAPTIVE (with supervisor + adaptive membership)

  fprintf ('=== STRESS TEST: Dense Cluttered Environment (Adaptive Law Evaluation) ===\n');

  P = fapf_params ('k_a', 20.0, 'k_r', 6.0, 'T_max', 240, 'dt', 5e-3);
  env = dense_env (120, 2026);  % Dense environment, slightly more obstacles
  nstart = 20;
  L = 7.0; Delta = 0.2; tauD = 0.6;

  % Original learning rates (not dampened)
  alpha_c_fast = 0.10;
  alpha_s_fast = 0.08;

  % Initialize trajectory storage
  traj_base    = cell (nstart, 1);
  traj_static  = cell (nstart, 1);
  traj_adapt   = cell (nstart, 1);

  results_base = struct ();
  results_static = struct ();
  results_adapt = struct ();

  rng (2026);
  random_angles = 2*pi * rand(nstart, 1);
  
  for i = 1:nstart
    ang = random_angles(i);
    q0  = env.goal + 0.85 * 9 * [cos(ang), sin(ang)];
    th0 = atan2 (env.goal(2)-q0(2), env.goal(1)-q0(1));

    % --- CASE 1: Base (no supervisor, no adaptation) ---
    out_b = fapf_sim_nonholo (q0, th0, env, P, struct ('adapt', false));
    traj_base{i} = out_b.traj;
    if (i == 1)
      results_base.success = zeros (1, nstart);
      results_base.t_goal = zeros (1, nstart);
      results_base.path_length = zeros (1, nstart);
      results_base.clr_min = zeros (1, nstart);
      results_base.clr_mean = zeros (1, nstart);
      results_base.clr_std = zeros (1, nstart);
      results_base.n_switches = zeros (1, nstart);
    end
    if (size (out_b.traj, 1) > 1)
      traj_diffs = diff (out_b.traj);
      path_len_b = sum (sqrt (sum (traj_diffs.^2, 2)));
    else
      path_len_b = 0;
    end
    results_base.success(i) = out_b.success;
    results_base.t_goal(i) = out_b.t_end;
    results_base.path_length(i) = path_len_b;
    results_base.clr_min(i) = out_b.dmin;
    results_base.clr_mean(i) = mean (out_b.dmin);
    results_base.clr_std(i) = 0;
    results_base.n_switches(i) = 0;

    % --- CASE 2: Adaptive only (WITH adaptation, but NO supervisor) ---
    out_s = run_adaptive_only_nonholo (q0, th0, env, P, alpha_c_fast, alpha_s_fast);
    traj_static{i} = out_s.traj;
    if (i == 1)
      results_static.success = zeros (1, nstart);
      results_static.t_goal = zeros (1, nstart);
      results_static.path_length = zeros (1, nstart);
      results_static.clr_min = zeros (1, nstart);
      results_static.clr_mean = zeros (1, nstart);
      results_static.clr_std = zeros (1, nstart);
      results_static.n_switches = zeros (1, nstart);
    end
    results_static.success(i) = out_s.success;
    results_static.t_goal(i) = out_s.t_end;
    results_static.path_length(i) = out_s.path_length;
    results_static.clr_min(i) = out_s.dmin;
    results_static.clr_mean(i) = out_s.clearance_mean;
    results_static.clr_std(i) = out_s.clearance_std;
    results_static.n_switches(i) = out_s.nsw;

    % --- CASE 3: Adaptive+Supervisor (full system) ---
    out_a = run_supervised_nonholo (q0, th0, env, P, true, L, Delta, tauD, alpha_c_fast, alpha_s_fast);
    traj_adapt{i} = out_a.traj;
    if (i == 1)
      results_adapt.success = zeros (1, nstart);
      results_adapt.t_goal = zeros (1, nstart);
      results_adapt.path_length = zeros (1, nstart);
      results_adapt.clr_min = zeros (1, nstart);
      results_adapt.clr_mean = zeros (1, nstart);
      results_adapt.clr_std = zeros (1, nstart);
      results_adapt.n_switches = zeros (1, nstart);
      adapt_centers = [];
      adapt_time = [];
    end
    results_adapt.success(i) = out_a.success;
    results_adapt.t_goal(i) = out_a.t_end;
    results_adapt.path_length(i) = out_a.path_length;
    results_adapt.clr_min(i) = out_a.dmin;
    results_adapt.clr_mean(i) = out_a.clearance_mean;
    results_adapt.clr_std(i) = out_a.clearance_std;
    results_adapt.n_switches(i) = out_a.nsw;

    if (isempty (adapt_time) || numel (out_a.time_hist) > numel (adapt_time))
      adapt_time    = out_a.time_hist;
      adapt_centers = out_a.centers_hist;
    end

    printf ('start %2d: base %-9s | adaptive-only %-7s | adaptive+sup %-9s\n', i, ...
            ternary_ (out_b.success, 'OK', 'FAIL'), ...
            ternary_ (out_s.success, 'OK', 'FAIL'), ...
            ternary_ (out_a.success, 'OK', 'FAIL'));
  end

  % ====== RESULTS SUMMARY TABLE ======
  printf ('\n========== RESULTS SUMMARY (20 trials, dense environment) ==========\n');
  printf ('%-18s | %-9s %-10s %-11s %-9s %-9s %-9s\n', ...
          'Configuration', 'Conv Rate', 'Time[s]', 'PathLen[m]', 'MinClr[m]', 'AvgClr[m]', 'N_Switch');
  printf (repmat ('-', 1, 100)); printf ('\n');

  % Base results
  base_conv = 100 * sum (results_base.success) / nstart;
  base_time = mean (results_base.t_goal(results_base.success > 0.5));
  base_path = mean (results_base.path_length(results_base.success > 0.5));
  base_minclr = min (results_base.clr_min);
  base_avgclr = mean (results_base.clr_mean(results_base.success > 0.5));
  printf ('Base (no sup)       | %6.1f%% | %8.2f | %10.2f | %8.4f | %8.4f |  N/A\n', ...
          base_conv, base_time, base_path, base_minclr, base_avgclr);

  % Adaptive-only results
  stat_conv = 100 * sum (results_static.success) / nstart;
  stat_minclr = min (results_static.clr_min);
  stat_time = mean (results_static.t_goal(results_static.success > 0.5));
  stat_path = mean (results_static.path_length(results_static.success > 0.5));
  stat_avgclr = mean (results_static.clr_mean(results_static.success > 0.5));
  printf ('Adaptive only       | %6.1f%% | %8.2f | %10.2f | %8.4f | %8.4f |        0\n', ...
          stat_conv, stat_time, stat_path, stat_minclr, stat_avgclr);

  % Adaptive+Supervisor results
  adap_conv = 100 * sum (results_adapt.success) / nstart;
  adap_minclr = min (results_adapt.clr_min);
  adap_time = mean (results_adapt.t_goal(results_adapt.success > 0.5));
  adap_path = mean (results_adapt.path_length(results_adapt.success > 0.5));
  adap_avgclr = mean (results_adapt.clr_mean(results_adapt.success > 0.5));
  adap_nsw = sum (results_adapt.n_switches) / nstart;
  printf ('Adaptive+Supervisor | %6.1f%% | %8.2f | %10.2f | %8.4f | %8.4f | %7.0f\n', ...
          adap_conv, adap_time, adap_path, adap_minclr, adap_avgclr, adap_nsw);

  % Improvements
  printf ('\n========== ADAPTIVE LAW EFFECTIVENESS (Dense Environment) ==========\n');
  fprintf ('Convergence improvement (Base vs Adaptive+Sup): %6.1f%%\n', adap_conv - base_conv);
  fprintf ('Time reduction (Base vs Adaptive+Sup):          %6.1f%%\n', 100*(base_time - adap_time)/base_time);
  fprintf ('Path length comparison:                        Base %.1fm vs Adaptive+Sup %.1fm\n', base_path, adap_path);
  
end

function s = ternary_ (cond, trueval, falseval)
  if (cond), s = trueval; else s = falseval; end
end
