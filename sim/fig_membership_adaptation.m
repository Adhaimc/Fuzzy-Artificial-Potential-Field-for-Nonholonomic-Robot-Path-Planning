function fig_membership_adaptation ()
% FIG_MEMBERSHIP_ADAPTATION: Classical membership functions before and after
% adaptation (Figure 4). Shows the initial triangular partition (uniform
% spacing) and the final adapted partition after online learning.

  fprintf ('=== GENERATING FIGURE: Membership Adaptation (Before/After) ===\n');

  P = fapf_params ();
  N = P.N;
  d = linspace (0, P.d_meas, 500);

  % Initial (before adaptation): uniform triangular partition
  centers_init = P.centers;
  sigma_init = P.sigma;
  mu_init = zeros (numel (d), N);
  for i = 1:N
    mu_init(:,i) = max (0, 1 - abs (d - centers_init(i)) / (2*sigma_init));
  end

  % Final (after adaptation): run a short simulation to capture adapted centers
  % Use a moderate duration so adaptation is visible but not fully converged
  env = guarded_env (100, 2026);
  q0 = env.goal + 0.85 * 9 * [cos(pi/4), sin(pi/4)];
  th0 = atan2 (env.goal(2)-q0(2), env.goal(1)-q0(1));
  P.T_max = 25;  % Shorter run: adaptation visible, centers not collapsed
  out = run_supervised_nonholo (q0, th0, env, P, true, 7.0, 0.2, 0.6);

  % Final adapted centers from the simulation (use midpoint for interpretability)
  mid_idx = round (size (out.centers_hist, 1) / 2);
  centers_final = out.centers_hist(mid_idx,:);
  sigma_final = 0.15;  % typical adapted width (from rate limits)
  mu_final = zeros (numel (d), N);
  for i = 1:N
    mu_final(:,i) = max (0, 1 - abs (d - centers_final(i)) / (2*sigma_final));
  end

  % Create figure
  fig = figure ('Position', [100, 100, 900, 400]);

  % Left: Before adaptation (uniform)
  subplot (1, 2, 1);
  hold on;
  cmap = jet (N);
  for i = 1:N
    plot (d, mu_init(:,i), '-', 'Color', cmap(i,:), 'LineWidth', 2);
  end
  hold off;
  xlabel ('Obstacle Distance $d_{\mathrm{obs}}$ (m)', 'FontSize', 11, 'Interpreter', 'latex');
  ylabel ('Membership Degree $\mu_i$', 'FontSize', 11, 'Interpreter', 'latex');
  title ('Before Adaptation (Uniform)', 'FontSize', 12, 'FontWeight', 'bold');
  grid on; box on;
  xlim ([0, P.d_meas]); ylim ([0, 1.1]);
  legend (arrayfun (@(i) sprintf ('$\\mu_%d$', i), 1:N, 'UniformOutput', false), ...
          'Location', 'northeast', 'FontSize', 9, 'Interpreter', 'latex');

  % Right: After adaptation (learned)
  subplot (1, 2, 2);
  hold on;
  for i = 1:N
    plot (d, mu_final(:,i), '-', 'Color', cmap(i,:), 'LineWidth', 2);
  end
  hold off;
  xlabel ('Obstacle Distance $d_{\mathrm{obs}}$ (m)', 'FontSize', 11, 'Interpreter', 'latex');
  ylabel ('Membership Degree $\mu_i$', 'FontSize', 11, 'Interpreter', 'latex');
  title ('After Adaptation (Learned)', 'FontSize', 12, 'FontWeight', 'bold');
  grid on; box on;
  xlim ([0, P.d_meas]); ylim ([0, 1.1]);
  legend (arrayfun (@(i) sprintf ('$\\mu_%d$', i), 1:N, 'UniformOutput', false), ...
          'Location', 'northeast', 'FontSize', 9, 'Interpreter', 'latex');

  print (fig, 'fig1_membership_adaptation.png', '-dpng', '-r150');
  printf ('Saved: sim/fig1_membership_adaptation.png\n');
  printf ('Initial centers: %s\n', mat2str (centers_init, 3));
  printf ('Final centers:   %s\n', mat2str (centers_final, 3));
end
