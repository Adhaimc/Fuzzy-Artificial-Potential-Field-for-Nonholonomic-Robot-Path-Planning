function plot_all (varargin)
% Generate publication-quality figures from E1–E6 experiments.
% Usage: plot_all('outdir', '/path/to/figs', 'fmt', 'pdf')
% Default: figs/ subdirectory, PDF format.

  opt = struct ('outdir', fullfile (pwd, 'figs'), 'fmt', 'png', ...
                'seeds', 1:2, 'nstart', 3, 'nobs', 25, ...
                'dt', 5e-3, 'T_max', 25);
  for i = 1:2:numel (varargin), opt.(varargin{i}) = varargin{i+1}; end

  if ~isfolder (opt.outdir), mkdir (opt.outdir); end
  
  % Use the headless-compatible toolkit used by the older plotting scripts.
  try, graphics_toolkit ('gnuplot'); catch, end
  
  fprintf ('=== Generating figures ===\n');
  fprintf ('Output: %s\n', opt.outdir);
  fprintf ('Format: %s\n\n', opt.fmt);

  % E1: Theory constants
  fprintf ('E1...\n');
  plot_e1 (opt);

  % E2: Equilibrium localisation
  fprintf ('E2...\n');
  plot_e2 (opt);

  % E3: Safety and barrier
  fprintf ('E3...\n');
  plot_e3 (opt);

  % E4: Penetration vs timestep
  fprintf ('E4...\n');
  plot_e4 (opt);

  % E5: Subgoal horizon ablation
  fprintf ('E5...\n');
  plot_e5 (opt);

  % E6: Adaptive membership
  fprintf ('E6...\n');
  plot_e6 (opt);

  fprintf ('\nFigures saved to: %s\n', opt.outdir);
end

function plot_e1 (opt)
  P = fapf_params ();
  d = linspace (P.r_safety, P.d0, 200);
  w1 = []; w2 = [];
  for i = 1:numel (d)
    [w1(i), w2(i)] = fapf_weights (d(i), P);
  end
  phi = w2 .* P.k_r .* fapf_gradUr_mag (d, P) + ...
        P.alpha0 .* (d - P.r_safety) - w1 .* P.k_a .* P.D_max;

  fig = figure ('visible', 'off', 'position', [0 0 900 600]);
  
  subplot (2,2,1);
  plot (d, phi, 'b-', 'linewidth', 2);
  hline (0, 'color', 'r', 'linestyle', '--', 'linewidth', 1.5);
  xlabel ('$d_{\rm obs}$ (m)', 'interpreter', 'latex', 'fontsize', 11);
  ylabel ('$\varphi(d)$ (m/s)', 'interpreter', 'latex', 'fontsize', 11);
  title (sprintf ('Barrier condition ($\\alpha_0 = %.2f$ s$^{-1}$)', P.alpha0), ...
         'interpreter', 'latex', 'fontsize', 12);
  grid on; set (gca, 'fontsize', 10);
  xlim ([P.r_safety, P.d0]);

  subplot (2,2,2);
  plot (d, w1, 'b-', 'linewidth', 2, 'displayname', '$w_1(d)$');
  hold on; plot (d, w2, 'r-', 'linewidth', 2, 'displayname', '$w_2(d)$');
  xlabel ('$d_{\rm obs}$ (m)', 'interpreter', 'latex', 'fontsize', 11);
  ylabel ('Weight', 'interpreter', 'latex', 'fontsize', 11);
  title ('Fuzzy membership functions', 'interpreter', 'latex', 'fontsize', 12);
  legend ('$w_1(d)$', '$w_2(d)$');
  grid on; set (gca, 'fontsize', 10);
  xlim ([P.r_safety, P.d0]);

  subplot (2,2,3);
  ratios = [0.6, 1.36, 2.5, 4.8, 10];
  k_as = ratios * P.k_r;
  R_vals = P.w2_max * P.k_r * P.G_r ./ (P.w1_min * k_as);
  semilogx (ratios, R_vals, 'bo-', 'linewidth', 2, 'markersize', 8);
  hline (9, 'color', 'r', 'linestyle', '--', 'linewidth', 1.5, ...
           'displayname', 'workspace radius');
  ylabel ('$R$ (m)', 'interpreter', 'latex', 'fontsize', 11);
  xlabel ('$k_a / k_r$', 'interpreter', 'latex', 'fontsize', 11);
  title ('Residual radius vs gain ratio', 'interpreter', 'latex', 'fontsize', 12);
  legend ('residual radius', 'workspace radius');
  grid on; set (gca, 'fontsize', 10);
  set (gca, 'xscale', 'log');

  subplot (2,2,4);
  y_conv = 4.808; y_safe_global = 2.137; y_safe_subgoal = 6.410;
  bounds = [y_conv, y_safe_global, y_safe_subgoal];
  bar (1:3, bounds, 'facecolor', [0.2 0.4 0.8], 'edgecolor', 'k', 'linewidth', 1.5);
  set (gca, 'xtick', 1:3, 'xticklabel', {'convergence', 'safety (global)', 'safety (L=3m)'});
  hline (0.6, 'color', 'r', 'linestyle', '--', 'linewidth', 1.5, ...
           'displayname', 'implemented ratio');
  ylabel ('$k_a / k_r$ bound', 'interpreter', 'latex', 'fontsize', 11);
  title ('Design window structure', 'interpreter', 'latex', 'fontsize', 12);
  grid on; set (gca, 'fontsize', 10); set (gca, 'ygrid', 'on', 'xgrid', 'off');
  ylim ([0, 8]);
  legend ('interpreter', 'latex', 'fontsize', 10, 'location', 'northeast');

  sgtitle_compat (fig, 'E1: Theory Constants Verification', 'fontsize', 14, 'fontweight', 'bold');
  print (fullfile (opt.outdir, ['E1_constants.' opt.fmt]), ['-d' opt.fmt], '-r150');
  close (fig);
end

function plot_e2 (opt)
  fig = figure ('visible', 'off', 'position', [0 0 1200 500]);

  subplot (1,2,1);
  ratios = [0.60; 1.36; 2.50; 4.80; 10.00];
  max_eq = [8.969; 8.806; 4.479; 0.000; 0.000];
  semilogx (ratios, max_eq, 'bo-', 'linewidth', 2, 'markersize', 8);
  hline (9, 'color', 'r', 'linestyle', '--', 'linewidth', 1.5, ...
           'displayname', 'workspace radius');
  ylabel ('$\max |q^*|$ (m)', 'interpreter', 'latex', 'fontsize', 11);
  xlabel ('$k_a / k_r$', 'interpreter', 'latex', 'fontsize', 11);
  title ('Equilibrium localisation', 'interpreter', 'latex', 'fontsize', 12);
  legend ('equilibrium radius', 'workspace radius');
  grid on; set (gca, 'fontsize', 10);
  set (gca, 'xscale', 'log');

  subplot (1,2,2);
  n_eq = [163; 82; 4; 0; 0];
  semilogx (ratios, n_eq, 'ro-', 'linewidth', 2, 'markersize', 8);
  ylabel ('Number of equilibria (on safe set)', 'interpreter', 'latex', 'fontsize', 11);
  xlabel ('$k_a / k_r$', 'interpreter', 'latex', 'fontsize', 11);
  title ('Equilibrium count vs gain ratio', 'interpreter', 'latex', 'fontsize', 12);
  grid on; set (gca, 'fontsize', 10);
  set (gca, 'xscale', 'log', 'yscale', 'log');

  sgtitle_compat (fig, 'E2: Equilibrium Localisation & Gain Sweep', 'fontsize', 14, 'fontweight', 'bold');
  print (fullfile (opt.outdir, ['E2_localisation.' opt.fmt]), ['-d' opt.fmt], '-r150');
  close (fig);
end

function plot_e3 (opt)
  res_holo = e3_barrier ('seeds', 1, 'nstart', 2, 'nobs', 20, ...
                         'model', 'holo', 'dt', opt.dt, 'T_max', opt.T_max);
  res_nholo = e3_barrier ('seeds', 1, 'nstart', 2, 'nobs', 20, ...
                          'model', 'nonholo', 'dt', opt.dt, 'T_max', opt.T_max);

  env = fapf_env (20, 1, res_holo.P, 'random');
  q0 = env.goal + 0.9 * res_holo.P.D_max;
  out_holo = fapf_sim_holo (q0, env, res_holo.P);
  th0 = atan2 (env.goal(2)-q0(2), env.goal(1)-q0(1));
  out_nholo = fapf_sim_nonholo (q0, th0, env, res_nholo.P);

  fig = figure ('visible', 'off', 'position', [0 0 1200 600]);

  % Holonomic trajectory
  subplot (2,2,1);
  if ~isempty (out_holo.traj) && size (out_holo.traj, 2) >= 2
    plot (out_holo.traj(:,1), out_holo.traj(:,2), 'b-', 'linewidth', 1.5);
    hold on;
    plot (out_holo.traj(1,1), out_holo.traj(1,2), 'go', 'markersize', 8, ...
          'displayname', 'start');
    plot (out_holo.traj(end,1), out_holo.traj(end,2), 'r*', 'markersize', 12, ...
          'displayname', 'goal');
    axis equal; grid on;
    xlabel ('$x$ (m)', 'interpreter', 'latex', 'fontsize', 11);
    ylabel ('$y$ (m)', 'interpreter', 'latex', 'fontsize', 11);
    title (sprintf ('Holonomic trajectory (clearance: %.3f m)', out_holo.dmin), ...
           'interpreter', 'latex', 'fontsize', 11);
    legend ('start', 'goal');
  end
  set (gca, 'fontsize', 10);

  % Clearance over time
  subplot (2,2,2);
  if ~isempty (out_holo.log) && size (out_holo.log, 2) >= 5
    t = out_holo.log(:, 1);
    d_obs = out_holo.log(:, 5);
    plot (t, d_obs, 'b-', 'linewidth', 1.5, 'displayname', 'holo');
    hold on;
    if ~isempty (out_nholo.log) && size (out_nholo.log, 2) >= 5
      t_nh = out_nholo.log(:, 1);
      d_nh = out_nholo.log(:, 5);
      plot (t_nh, d_nh, 'r-', 'linewidth', 1.5, 'displayname', 'nonholo');
    end
    hline (0.5, 'color', 'k', 'linestyle', '--', 'linewidth', 1, ...
             'displayname', '$r_{\rm safety}$');
    xlabel ('Time (s)', 'interpreter', 'latex', 'fontsize', 11);
    ylabel ('$d_{\rm obs}$ (m)', 'interpreter', 'latex', 'fontsize', 11);
    title ('Obstacle distance over time', 'interpreter', 'latex', 'fontsize', 11);
        legend ('holo', 'nonholo', '$r_{safety}$');
    grid on;
  end
  set (gca, 'fontsize', 10);

  % Nonholonomic trajectory
  subplot (2,2,3);
  if ~isempty (out_nholo.traj) && size (out_nholo.traj, 2) >= 2
    plot (out_nholo.traj(:,1), out_nholo.traj(:,2), 'r-', 'linewidth', 1.5);
    hold on;
    plot (out_nholo.traj(1,1), out_nholo.traj(1,2), 'go', 'markersize', 8, ...
          'displayname', 'start');
    plot (out_nholo.traj(end,1), out_nholo.traj(end,2), 'r*', 'markersize', 12, ...
          'displayname', 'goal');
    axis equal; grid on;
    xlabel ('$x$ (m)', 'interpreter', 'latex', 'fontsize', 11);
    ylabel ('$y$ (m)', 'interpreter', 'latex', 'fontsize', 11);
    title (sprintf ('Nonholonomic trajectory (clearance: %.3f m)', out_nholo.dmin), ...
           'interpreter', 'latex', 'fontsize', 11);
    legend ('start', 'goal');
  end
  set (gca, 'fontsize', 10);

  % Barrier margin comparison
  subplot (2,2,4);
  cats = 1:2;
  margins_720 = [res_holo.holo.margin(1), res_nholo.nonholo.margin(1)];
  margins_871 = [res_holo.holo.margin(2), res_nholo.nonholo.margin(2)];
  b = bar (cats, [margins_720; margins_871]');
  ylabel ('Barrier margin (m)', 'interpreter', 'latex', 'fontsize', 11);
  title ('Safety margins at convergence', 'interpreter', 'latex', 'fontsize', 11);
    set (gca, 'xtick', cats, 'xticklabel', {'holo', 'nonholo'});
    legend ('$\alpha_0 = 7.20$ s$^{-1}$', '$\alpha_0 = 8.71$ s$^{-1}$');
  grid on; set (gca, 'fontsize', 10); set (gca, 'ygrid', 'on', 'xgrid', 'off');

  sgtitle_compat (fig, 'E3: Closed-Loop Safety & Barrier Verification', 'fontsize', 14, 'fontweight', 'bold');
  print (fullfile (opt.outdir, ['E3_safety.' opt.fmt]), ['-d' opt.fmt], '-r150');
  close (fig);
end

function plot_e4 (opt)
  res = e4_timestep ('dts', [1e-4 2e-4 5e-4 1e-3 2e-3 5e-3], ...
                     'seeds', 1, 'nobs', 20, 'T_max', 20);

  fig = figure ('visible', 'off', 'position', [0 0 1000 600]);

  dts = [1e-4 2e-4 5e-4 1e-3 2e-3 5e-3];

  % Setting A: implemented gains
  subplot (2,2,1);
  pen_a = res.implemented(:, 2);
  bound_a = res.implemented(:, 3);
  semilogy (dts, bound_a, 'b--', 'linewidth', 2, 'displayname', '$v_{\max}\Delta t$ bound');
  hold on;
  semilogy (dts, pen_a + eps, 'bo', 'linewidth', 2, 'markersize', 8, ...
            'displayname', 'penetration');
  xlabel ('$\Delta t$ (s)', 'interpreter', 'latex', 'fontsize', 11);
  ylabel ('Penetration (m)', 'interpreter', 'latex', 'fontsize', 11);
  title ('(A) Implemented gains', 'interpreter', 'latex', 'fontsize', 11);
  legend ('$v_{max} dt$ bound', 'penetration');
  grid on; set (gca, 'fontsize', 10);
  set (gca, 'xscale', 'log', 'yscale', 'log');

  % Setting B: near-critical gains
  subplot (2,2,2);
  pen_b = res.near_critical(:, 2);
  bound_b = res.near_critical(:, 3);
  semilogy (dts, bound_b, 'r--', 'linewidth', 2, 'displayname', '$v_{\max}\Delta t$ bound');
  hold on;
  semilogy (dts, pen_b + eps, 'ro', 'linewidth', 2, 'markersize', 8, ...
            'displayname', 'penetration');
  xlabel ('$\Delta t$ (s)', 'interpreter', 'latex', 'fontsize', 11);
  ylabel ('Penetration (m)', 'interpreter', 'latex', 'fontsize', 11);
  title ('(B) Near-critical gains', 'interpreter', 'latex', 'fontsize', 11);
  legend ('$v_{max} dt$ bound', 'penetration');
  grid on; set (gca, 'fontsize', 10);
  set (gca, 'xscale', 'log', 'yscale', 'log');

  % Setting C: supercritical gains
  subplot (2,2,3);
  pen_c = res.supercritical(:, 2);
  bound_c = res.supercritical(:, 3);
  semilogy (dts, bound_c, 'g--', 'linewidth', 2, 'displayname', '$v_{\max}\Delta t$ bound');
  hold on;
  semilogy (dts, pen_c + eps, 'go', 'linewidth', 2, 'markersize', 8, ...
            'displayname', 'penetration');
  xlabel ('$\Delta t$ (s)', 'interpreter', 'latex', 'fontsize', 11);
  ylabel ('Penetration (m)', 'interpreter', 'latex', 'fontsize', 11);
  title ('(C) Supercritical gains (safety FAILS)', 'interpreter', 'latex', 'fontsize', 11);
  legend ('$v_{max} dt$ bound', 'penetration');
  grid on; set (gca, 'fontsize', 10);
  set (gca, 'xscale', 'log', 'yscale', 'log');

  % Combined comparison
  subplot (2,2,4);
  semilogy (dts, pen_a + eps, 'bo-', 'linewidth', 2, 'markersize', 6, ...
            'displayname', '(A) implemented');
  hold on;
  semilogy (dts, pen_b + eps, 'ro-', 'linewidth', 2, 'markersize', 6, ...
            'displayname', '(B) near-critical');
  semilogy (dts, pen_c + eps, 'go-', 'linewidth', 2, 'markersize', 6, ...
            'displayname', '(C) supercritical');
  semilogy (dts, dts * 100, 'k--', 'linewidth', 1, ...
            'displayname', 'reference $O(\Delta t)$');
  xlabel ('$\Delta t$ (s)', 'interpreter', 'latex', 'fontsize', 11);
  ylabel ('Penetration (m)', 'interpreter', 'latex', 'fontsize', 11);
  title ('All settings: penetration vs step size', 'interpreter', 'latex', 'fontsize', 11);
  legend ('implemented', 'near-critical', 'supercritical', 'reference O(dt)');
  grid on; set (gca, 'fontsize', 10);
  set (gca, 'xscale', 'log', 'yscale', 'log');

  sgtitle_compat (fig, 'E4: Sampled-Data Penetration vs Integration Step', 'fontsize', 14, 'fontweight', 'bold');
  print (fullfile (opt.outdir, ['E4_penetration.' opt.fmt]), ['-d' opt.fmt], '-r150');
  close (fig);
end

function plot_e5 (opt)
  res = e5_subgoal ('seeds', opt.seeds, 'nstart', opt.nstart, 'nobs', opt.nobs, ...
                    'dt', opt.dt, 'T_max', opt.T_max);

  fig = figure ('visible', 'off', 'position', [0 0 1000 600]);

  subplot (2,2,1);
  L_vals = res.rows(:, 1);
  safe_bnd = res.rows(:, 2);
  semilogx (L_vals, safe_bnd, 'bo-', 'linewidth', 2, 'markersize', 8);
  hline (5.5, 'color', 'r', 'linestyle', '--', 'linewidth', 1.5, ...
           'displayname', 'implemented ratio');
  ylabel ('$k_a / k_r$ upper bound', 'interpreter', 'latex', 'fontsize', 11);
  xlabel ('Horizon $L$ (m)', 'interpreter', 'latex', 'fontsize', 11);
  title ('Safety bound vs horizon', 'interpreter', 'latex', 'fontsize', 11);
  legend ('implemented ratio', 'safe bound');
  grid on; set (gca, 'fontsize', 10);
  set (gca, 'xscale', 'log');

  subplot (2,2,2);
  succ = res.rows(:, 4);
  bar (L_vals, succ, 'facecolor', [0.2 0.6 0.2], 'edgecolor', 'k', 'linewidth', 1.5);
  ylabel ('Success rate (%)', 'interpreter', 'latex', 'fontsize', 11);
  xlabel ('Horizon $L$ (m)', 'interpreter', 'latex', 'fontsize', 11);
  title ('Goal reaching probability', 'interpreter', 'latex', 'fontsize', 11);
  ylim ([0, 110]); grid on; set (gca, 'fontsize', 10); set (gca, 'ygrid', 'on', 'xgrid', 'off');

  subplot (2,2,3);
  clr = res.rows(:, 5);
  bar (L_vals, clr, 'facecolor', [0.8 0.4 0.2], 'edgecolor', 'k', 'linewidth', 1.5);
  ylabel ('Min clearance (m)', 'interpreter', 'latex', 'fontsize', 11);
  xlabel ('Horizon $L$ (m)', 'interpreter', 'latex', 'fontsize', 11);
  title ('Minimum obstacle clearance', 'interpreter', 'latex', 'fontsize', 11);
  hline (0.5, 'color', 'r', 'linestyle', '--', 'linewidth', 1.5, ...
           'displayname', '$r_{\rm safety}$');
  grid on; set (gca, 'fontsize', 10); set (gca, 'ygrid', 'on', 'xgrid', 'off');

  subplot (2,2,4);
  win = res.rows(:, 3);  % IN=1, OUT=0
  bar (L_vals, win, 'facecolor', [0.2 0.4 0.8], 'edgecolor', 'k', 'linewidth', 1.5);
  ylabel ('In design window?', 'interpreter', 'latex', 'fontsize', 11);
  xlabel ('Horizon $L$ (m)', 'interpreter', 'latex', 'fontsize', 11);
  title ('Convergence + safety coexistence', 'interpreter', 'latex', 'fontsize', 11);
  ylim ([-0.5, 1.5]); set (gca, 'ytick', [0, 1], 'yticklabel', {'NO', 'YES'});
  grid on; set (gca, 'fontsize', 10); set (gca, 'ygrid', 'on', 'xgrid', 'off');

  sgtitle_compat (fig, 'E5: Sub-Goal Horizon Ablation (GNRON)', 'fontsize', 14, 'fontweight', 'bold');
  print (fullfile (opt.outdir, ['E5_subgoal.' opt.fmt]), ['-d' opt.fmt], '-r150');
  close (fig);
end

function plot_e6 (opt)
  % E6 is more complex; show console output and summary
  fig = figure ('visible', 'off', 'position', [0 0 800 600]);
  axis off;
  lines = {'E6: Adaptive Membership Tuning', '', ...
           'Part (1): Convergence rate', ...
           '  Theory: rate-independent adaptation', ...
           '  Observed: -3.21% rate loss', ...
           '  Status: PASS', '', ...
           'Part (2): Steering-gain condition', ...
           '  D (OFF): 135.00 s^-1', ...
           '  D_adapt (ON): 410.22 s^-1', ...
           '  Bound: 156.97 s^-1', ...
           '  Status: VIOLATED', '', ...
           'Part (3): k_omega sweep', ...
           '  Required: 1427.62 s^-1', ...
           '  Implemented: 3.00 s^-1', ...
           '  Gap: 475x too small', '', ...
           'Run full simulations for details.'};
  y = 0.94;
  for i = 1:numel (lines)
    text (0.08, y, lines{i}, 'interpreter', 'none', 'fontsize', 11, ...
          'horizontalalignment', 'left', 'fontname', 'monospace');
    y = y - 0.045;
  end
  title ('E6: Adaptive Membership Tuning', 'fontsize', 14, 'fontweight', 'bold');
  print (fullfile (opt.outdir, ['E6_adaptive.' opt.fmt]), ['-d' opt.fmt], '-r150');
  close (fig);
end

function s = ternary (c, a, b)
  if (c), s = a; else, s = b; end
end

function h = hline (y, varargin)
  xl = xlim ();
  h = plot (xl, [y y], varargin{:});
end

function sgtitle_compat (fig, txt, varargin)
  axes ('parent', fig, 'position', [0 0 1 1], 'visible', 'off');
  text (0.5, 0.985, txt, 'units', 'normalized', ...
        'horizontalalignment', 'center', varargin{:});
end
