function fig_stress_test ()
% FIG_STRESS_TEST: "Guarded goal" stress test (simplified to BASE vs ADAPTIVE comparison).
% The goal is enclosed by a ring of obstacles with a single narrow gate,
% inside a dense 100-obstacle field. Performs a TWO-WAY comparison:
%   1. BASE (no adaptation)
%   2. ADAPTIVE (Proposal 2 with Lyapunov feedback - SAFE tuning)
%
% This compares the raw effectiveness of the adaptive law without supervisor interference.

  fprintf ('=== STRESS TEST: Guarded Goal (BASE vs ADAPTIVE, 100 obstacles, 20 trials) ===\n');
  fprintf ('Both using boosted gains (k_a=20, k_r=6) to test adaptive improvement\n\n');

  % Use same boosted gains for both base and adaptive
  P = fapf_params ('k_a', 20.0, 'k_r', 6.0, 'T_max', 240, 'dt', 5e-3);
  env = guarded_env (100, 2026);
  nstart = 20;

  % Initialize trajectory storage for 2-way comparison
  traj_base = cell (nstart, 1);
  traj_adapt = cell (nstart, 1);

  % Storage for metrics
  results_base = struct ();
  results_adapt = struct ();

  % Randomize initial placements around perimeter
  rng (2026);  % Fixed seed for reproducibility
  random_angles = 2*pi * rand(nstart, 1);
  
  for i = 1:nstart
    ang = random_angles(i);
    q0  = env.goal + 0.85 * 9 * [cos(ang), sin(ang)];
    th0 = atan2 (env.goal(2)-q0(2), env.goal(1)-q0(1));

    % --- CASE 1: Base (no adaptation) with boosted gains ---
    out_b = fapf_sim_nonholo (q0, th0, env, P, struct ('adapt', false));
    traj_base{i} = out_b.traj;
    if (i == 1)
      results_base.success = zeros (1, nstart);
      results_base.t_goal = zeros (1, nstart);
      results_base.path_length = zeros (1, nstart);
      results_base.clr_min = zeros (1, nstart);
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

    % --- CASE 2: Adaptive with same boosted gains ---
    out_a = run_adaptive_only_nonholo (q0, th0, env, P);
    traj_adapt{i} = out_a.traj;
    if (i == 1)
      results_adapt.success = zeros (1, nstart);
      results_adapt.t_goal = zeros (1, nstart);
      results_adapt.path_length = zeros (1, nstart);
      results_adapt.clr_min = zeros (1, nstart);
    end
    results_adapt.success(i) = out_a.success;
    results_adapt.t_goal(i) = out_a.t_end;
    results_adapt.path_length(i) = out_a.path_length;
    results_adapt.clr_min(i) = out_a.dmin;

    printf ('Trial %2d: Base (k_a=20) %-7s | Adaptive (k_a=20) %-7s\n', i, ...
            ternary_ (out_b.success, 'OK', 'FAIL'), ...
            ternary_ (out_a.success, 'OK', 'FAIL'));
  end

  % ====== RESULTS SUMMARY TABLE (2-WAY COMPARISON) ======
  printf ('\n============ RESULTS SUMMARY (20 trials, guarded-goal environment) ============\n');
  printf ('%-18s | Conv Rate | Time[s] | PathLen[m] | MinClr[m]\n');
  printf (repmat ('-', 1, 70)); printf ('\n');

  % Base results
  base_conv = 100 * sum (results_base.success) / nstart;
  base_succ_idx = results_base.success > 0.5;
  if (sum (base_succ_idx) > 0)
    base_time = mean (results_base.t_goal(base_succ_idx));
    base_path = mean (results_base.path_length(base_succ_idx));
  else
    base_time = nan; base_path = nan;
  end
  base_minclr = min (results_base.clr_min);
  printf ('Base (no adapt)     | %6.1f%% | %7.2f | %10.2f | %8.4f\n', ...
          base_conv, base_time, base_path, base_minclr);

  % Adaptive results
  adapt_conv = 100 * sum (results_adapt.success) / nstart;
  adapt_succ_idx = results_adapt.success > 0.5;
  if (sum (adapt_succ_idx) > 0)
    adapt_time = mean (results_adapt.t_goal(adapt_succ_idx));
    adapt_path = mean (results_adapt.path_length(adapt_succ_idx));
  else
    adapt_time = nan; adapt_path = nan;
  end
  adapt_minclr = min (results_adapt.clr_min);
  printf ('Adaptive (Prop 2)   | %6.1f%% | %7.2f | %10.2f | %8.4f\n', ...
          adapt_conv, adapt_time, adapt_path, adapt_minclr);

  % Summary insights
  printf ('\n========== SUMMARY ==========\n');
  printf ('Base convergence rate:           %.1f%%\n', base_conv);
  printf ('Adaptive convergence rate:       %.1f%%\n', adapt_conv);
  if (adapt_conv > base_conv)
    printf ('IMPROVEMENT:                     +%.1f%% points\n', adapt_conv - base_conv);
  else
    printf ('REGRESSION:                      -%.1f%% points\n', base_conv - adapt_conv);
  end
  printf ('Completed: %d/%d trials\n\n', nstart, nstart);

  % Two-way trajectory comparison (Figure 2: side-by-side panels)
  printf ('\n=== GENERATING FIGURE: Base vs Adaptive Trajectory Comparison ===\n');
  fig = figure ('Position', [100, 100, 1400, 650]);
  clf;

  % Left panel: Base trajectories
  ax1 = axes ('Parent', fig, 'Position', [0.08, 0.12, 0.4, 0.75]);
  axes(ax1);
  draw_panel (env, P, traj_base, results_base.success, 'Base (No Adaptation)');

  % Right panel: Adaptive trajectories
  ax2 = axes ('Parent', fig, 'Position', [0.52, 0.12, 0.4, 0.75]);
  axes(ax2);
  draw_panel (env, P, traj_adapt, results_adapt.success, 'Adaptive (Proposal 2)');

  print (fig, 'fig2_overlay_10trajectories.png', '-dpng', '-r75');
  printf ('Saved: sim/fig2_overlay_10trajectories.png\n');
end

% =====================================================================

function draw_panel (env, P, traj_set, conv_flags, title_str)
  hold on; axis equal;

  theta_circ = linspace (0, 2*pi, 32);
  for i = 1:size (env.obs, 1)
    ox = env.obs(i,1); oy = env.obs(i,2); orad = env.obs(i,3);
    patch (ox + orad*cos(theta_circ), oy + orad*sin(theta_circ), ...
           [0.7, 0.7, 0.7], 'EdgeColor', 'k', 'LineWidth', 0.5);
  end

  cmap = hsv (numel (traj_set));
  for i = 1:numel (traj_set)
    traj = traj_set{i};
    if (isempty (traj) || size (traj, 1) <= 1), continue; end

    plot (traj(:,1), traj(:,2), '-', 'Color', cmap(i,:), 'LineWidth', 1.5);

    arrow_spacing = max (5, round (size (traj, 1) / 4));
    for k = 1:arrow_spacing:size(traj,1)-1
      dx = traj(k+1,1) - traj(k,1);
      dy = traj(k+1,2) - traj(k,2);
      norm_d = sqrt (dx^2 + dy^2) + 1e-6;
      heading_angle = atan2 (dy, dx);

      robot_size = 0.3;
      corners_x = robot_size * [-0.5, 0.5, 0.5, -0.5];
      corners_y = robot_size * [-0.5, -0.5, 0.5, 0.5];
      cos_h = cos (heading_angle); sin_h = sin (heading_angle);
      rx = corners_x*cos_h - corners_y*sin_h + traj(k,1);
      ry = corners_x*sin_h + corners_y*cos_h + traj(k,2);
      patch (rx, ry, 'k');

      quiver (traj(k,1), traj(k,2), dx/norm_d*0.5, dy/norm_d*0.5, 0, ...
              'Color', 'k', 'LineWidth', 1.2, 'MaxHeadSize', 0.4);
    end

    plot (traj(1,1), traj(1,2), 'o', 'Color', cmap(i,:), 'MarkerSize', 5, 'MarkerFaceColor', cmap(i,:));

    if (conv_flags(i) && size (traj, 1) > 1)
      final_pos = traj(end, 1:2);
      dx = traj(end,1) - traj(end-1,1);
      dy = traj(end,2) - traj(end-1,2);
      heading_angle = atan2 (dy, dx);
      robot_size = 0.4;
      corners_x = robot_size * [-0.5, 0.5, 0.5, -0.5];
      corners_y = robot_size * [-0.5, -0.5, 0.5, 0.5];
      cos_h = cos (heading_angle); sin_h = sin (heading_angle);
      rx = corners_x*cos_h - corners_y*sin_h + final_pos(1);
      ry = corners_x*sin_h + corners_y*cos_h + final_pos(2);
      patch (rx, ry, [0, 0.8, 0], 'EdgeColor', [0, 0.5, 0], 'LineWidth', 1.5);
    end
  end

  plot (env.goal(1), env.goal(2), '*', 'Color', [0, 0.7, 0], 'MarkerSize', 20, 'LineWidth', 2);

  set (gca, 'XLim', [-10, 10], 'YLim', [-10, 10]);
  set (gca, 'XTick', [-10:5:10], 'YTick', [-10:5:10]);
  grid off; box on; axis equal;
  xlabel ('X (m)', 'FontSize', 11);
  ylabel ('Y (m)', 'FontSize', 11);
  title (title_str, 'FontSize', 12, 'FontWeight', 'bold');
end

function draw_panel_overlay (env, P, traj_base, traj_static, conv_base, conv_static, nstart)
  % Overlay base and static+supervisor trajectories (simplified to reduce file size)
  hold on; axis equal;

  % Draw obstacles (gray circles only, no d0 range circles)
  theta_circ = linspace (0, 2*pi, 32);
  for i = 1:size (env.obs, 1)
    ox = env.obs(i,1); oy = env.obs(i,2); orad = env.obs(i,3);
    % Obstacle body (solid gray)
    patch (ox + orad*cos(theta_circ), oy + orad*sin(theta_circ), ...
           [0.7, 0.7, 0.7], 'EdgeColor', 'k', 'LineWidth', 0.5);
  end

  % Draw base trajectories with rainbow colormap and robot shapes
  cmap = hsv(numel (traj_base));
  for i = 1:numel (traj_base)
    traj = traj_base{i};
    if (isempty (traj) || size (traj, 1) <= 1), continue; end
    
    % Trajectory line
    plot (traj(:,1), traj(:,2), '-', 'Color', cmap(i,:), 'LineWidth', 1.5);
    
    % Robot shapes and arrows along trajectory
    arrow_spacing = max (5, round (size(traj, 1) / 4));
    for k = 1:arrow_spacing:size(traj,1)-1
      dx = traj(k+1,1) - traj(k,1);
      dy = traj(k+1,2) - traj(k,2);
      norm_d = sqrt (dx^2 + dy^2) + 1e-6;
      heading_angle = atan2 (dy, dx);

      % Draw robot shape
      robot_size = 0.3;
      corners_x = robot_size * [-0.5, 0.5, 0.5, -0.5];
      corners_y = robot_size * [-0.5, -0.5, 0.5, 0.5];
      cos_h = cos (heading_angle); sin_h = sin (heading_angle);
      rx = corners_x*cos_h - corners_y*sin_h + traj(k,1);
      ry = corners_x*sin_h + corners_y*cos_h + traj(k,2);
      patch (rx, ry, cmap(i,:));

      % Draw heading arrow
      quiver (traj(k,1), traj(k,2), dx/norm_d*0.4, dy/norm_d*0.4, 0, ...
              'Color', 'k', 'LineWidth', 1.0, 'MaxHeadSize', 0.3);
    end

    % Mark starting position
    plot (traj(1,1), traj(1,2), 'o', 'Color', cmap(i,:), 'MarkerSize', 5, 'MarkerFaceColor', cmap(i,:));

    % Mark final position with larger green robot if converged
    if (conv_base(i) && size (traj, 1) > 1)
      final_pos = traj(end, 1:2);
      dx = traj(end,1) - traj(end-1,1);
      dy = traj(end,2) - traj(end-1,2);
      heading_angle = atan2 (dy, dx);
      robot_size = 0.4;
      corners_x = robot_size * [-0.5, 0.5, 0.5, -0.5];
      corners_y = robot_size * [-0.5, -0.5, 0.5, 0.5];
      cos_h = cos (heading_angle); sin_h = sin (heading_angle);
      rx = corners_x*cos_h - corners_y*sin_h + final_pos(1);
      ry = corners_x*sin_h + corners_y*cos_h + final_pos(2);
      patch (rx, ry, [0, 0.8, 0], 'EdgeColor', [0, 0.5, 0], 'LineWidth', 1.5);
    end
  end

  % Draw static+supervisor trajectories (these should fail - shown in gray dashed, no robots since 0% success)
  for i = 1:numel (traj_static)
    traj = traj_static{i};
    if (isempty (traj) || size (traj, 1) <= 1), continue; end
    plot (traj(:,1), traj(:,2), '--', 'Color', [0.5, 0.5, 0.5], 'LineWidth', 1.0);
  end

  % Goal marker (green star)
  plot (env.goal(1), env.goal(2), '*', 'Color', [0, 0.7, 0], 'MarkerSize', 20, 'LineWidth', 2);

  set (gca, 'XLim', [-10, 10], 'YLim', [-10, 10]);
  set (gca, 'XTick', [-10:5:10], 'YTick', [-10:5:10]);
  grid on; axis equal; box on;
  xlabel ('X (m)', 'FontSize', 11);
  ylabel ('Y (m)', 'FontSize', 11);
end

function plot_weight_functions (centers, sigma_val, P, weight_type)
  % Plot control weight functions w_1(d) or w_2(d) using membership zones
  % centers: vector of zone center locations
  % sigma_val: scalar width parameter (uniform for all zones)
  % P: parameters structure with w1c, w2c rule consequents
  % weight_type: 1 for w_1, 2 for w_2
  
  d_range = linspace (0, 3, 200);
  w_vals = zeros (size (d_range));
  
  % Compute weight function for each distance
  for j = 1:numel(d_range)
    d = d_range(j);
    mu = max (0, 1 - abs(d - centers) ./ (2 * sigma_val + 1e-6));
    s = sum (mu);
    if (s < 1e-12)
      % Outside partition support: clamp to nearest zone
      if (d <= centers(1))
        mu = zeros (size (centers)); mu(1) = 1; s = 1;
      else
        mu = zeros (size (centers)); mu(end) = 1; s = 1;
      end
    end
    
    if (weight_type == 1)
      w_vals(j) = sum (mu .* P.w1c) / s;
    else
      w_vals(j) = sum (mu .* P.w2c) / s;
    end
  end
  
  % Plot the weight function
  hold on;
  plot (d_range, w_vals, '-', 'Color', 'b', 'LineWidth', 2.5);
  hold off;
end

function len = compute_path_length (traj)
  % Compute total path length from trajectory, handling empty/short trajectories
  if isempty(traj) || size(traj, 1) <= 1
    len = 0;
  else
    diffs = diff(traj);
    len = sum(sqrt(sum(diffs.^2, 2)));
  end
end

function s = ternary_ (c, a, b)
  if (c), s = a; else, s = b; end
end
