function fig_membership_adaptation_fixed ()
% FIG_MEMBERSHIP_ADAPTATION_FIXED: Shows weighted consequents w_1 and w_2
% before and after adaptive width tuning.
% 
% Displays four panels:
%   Top-left: w_1(d) before adaptation (σ = 0.375)
%   Top-right: w_1(d) after adaptation (σ = 1.0)
%   Bottom-left: w_2(d) before adaptation (σ = 0.375)
%   Bottom-right: w_2(d) after adaptation (σ = 1.0)

  fprintf ('=== GENERATING FIGURE: Weighted Consequents w_1 and w_2 (Before/After) ===\n');

  P = fapf_params ();
  N = P.N;
  d = linspace (0, P.d_meas, 500);

  % BEFORE adaptation: fixed centers and initial width
  centers = P.centers;    % Fixed: [0, 0.75, 1.5, 2.25, 3.0]
  sigma_before = P.sigma; % Initial: 0.375
  
  % Compute membership functions (triangular)
  mu_before = zeros (numel (d), N);
  for i = 1:N
    mu_before(:,i) = max (0, 1 - abs (d - centers(i)) / (2*sigma_before));
  end
  
  % Compute weighted consequents: w_j(d) = sum_i mu_i(d) * w_j,i / sum_i mu_i(d)
  w1_before = (mu_before * P.w1_c(:)) ./ (sum(mu_before, 2) + 1e-6);
  w2_before = (mu_before * P.w2_c(:)) ./ (sum(mu_before, 2) + 1e-6);

  % AFTER adaptation: same fixed centers, but increased width
  sigma_after = 1.0;  % Adapted: 1.0 (upper limit from constraint verification)
  
  % Compute membership functions (triangular, wider zones)
  mu_after = zeros (numel (d), N);
  for i = 1:N
    mu_after(:,i) = max (0, 1 - abs (d - centers(i)) / (2*sigma_after));
  end
  
  % Compute weighted consequents
  w1_after = (mu_after * P.w1_c(:)) ./ (sum(mu_after, 2) + 1e-6);
  w2_after = (mu_after * P.w2_c(:)) ./ (sum(mu_after, 2) + 1e-6);

  % Create figure with 2x2 layout
  fig = figure ('Position', [100, 100, 1200, 900]);
  
  % === TOP-LEFT: w_1 before adaptation ===
  subplot (2, 2, 1);
  hold on;
  plot (d, w1_before, 'b-', 'LineWidth', 2.5, 'DisplayName', 'w_1(d)');
  plot (d, mu_before, ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5, 'DisplayName', '\mu_i(d) basis');
  hold off;
  xlabel ('Obstacle Distance d_{obs} (m)', 'FontSize', 11);
  ylabel ('Attractive Weight w_1(d)', 'FontSize', 11);
  title ('Before Adaptation: \sigma = 0.375 m', 'FontSize', 12, 'FontWeight', 'bold');
  grid on; box on;
  xlim ([0, P.d_meas]); ylim ([0.2, 1.05]);
  legend ('Location', 'southeast', 'FontSize', 9);
  
  % === TOP-RIGHT: w_1 after adaptation ===
  subplot (2, 2, 2);
  hold on;
  plot (d, w1_after, 'b-', 'LineWidth', 2.5, 'DisplayName', 'w_1(d)');
  plot (d, mu_after, ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5, 'DisplayName', '\mu_i(d) basis');
  hold off;
  xlabel ('Obstacle Distance d_{obs} (m)', 'FontSize', 11);
  ylabel ('Attractive Weight w_1(d)', 'FontSize', 11);
  title ('After Adaptation: \sigma = 1.0 m', 'FontSize', 12, 'FontWeight', 'bold');
  grid on; box on;
  xlim ([0, P.d_meas]); ylim ([0.2, 1.05]);
  legend ('Location', 'southeast', 'FontSize', 9);
  
  % === BOTTOM-LEFT: w_2 before adaptation ===
  subplot (2, 2, 3);
  hold on;
  plot (d, w2_before, 'r-', 'LineWidth', 2.5, 'DisplayName', 'w_2(d)');
  plot (d, mu_before, ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5, 'DisplayName', '\mu_i(d) basis');
  hold off;
  xlabel ('Obstacle Distance d_{obs} (m)', 'FontSize', 11);
  ylabel ('Repulsive Weight w_2(d)', 'FontSize', 11);
  title ('Before Adaptation: \sigma = 0.375 m', 'FontSize', 12, 'FontWeight', 'bold');
  grid on; box on;
  xlim ([0, P.d_meas]); ylim ([0.15, 1.05]);
  legend ('Location', 'southeast', 'FontSize', 9);
  
  % === BOTTOM-RIGHT: w_2 after adaptation ===
  subplot (2, 2, 4);
  hold on;
  plot (d, w2_after, 'r-', 'LineWidth', 2.5, 'DisplayName', 'w_2(d)');
  plot (d, mu_after, ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5, 'DisplayName', '\mu_i(d) basis');
  hold off;
  xlabel ('Obstacle Distance d_{obs} (m)', 'FontSize', 11);
  ylabel ('Repulsive Weight w_2(d)', 'FontSize', 11);
  title ('After Adaptation: \sigma = 1.0 m', 'FontSize', 12, 'FontWeight', 'bold');
  grid on; box on;
  xlim ([0, P.d_meas]); ylim ([0.15, 1.05]);
  legend ('Location', 'southeast', 'FontSize', 9);

  print (fig, 'fig1_membership_adaptation.png', '-dpng', '-r150');
  fprintf ('Saved: sim/fig1_membership_adaptation.png\n');
  fprintf ('Configuration:\n');
  fprintf ('  Fixed centers: [%.2f, %.2f, %.2f, %.2f, %.2f] m\n', centers);
  fprintf ('  Before: sigma = %.4f m\n', sigma_before);
  fprintf ('  After:  sigma = %.4f m\n', sigma_after);
  fprintf ('  w_1 range: [%.3f, %.3f] (before) -> [%.3f, %.3f] (after)\n', ...
           min(w1_before), max(w1_before), min(w1_after), max(w1_after));
  fprintf ('  w_2 range: [%.3f, %.3f] (before) -> [%.3f, %.3f] (after)\n', ...
           min(w2_before), max(w2_before), min(w2_after), max(w2_after));
  
end
