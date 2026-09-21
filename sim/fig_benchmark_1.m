function fig_benchmark_1()
% FIG_BENCHMARK_1: Generate publication figure for Benchmark I (Holonomic)
% Shows trajectory comparison: Fuzzy-APF (Left) vs Classical Khatib (Right)
% Display: 20 trajectories overlaid, 100 obstacles, goal marker

  fprintf('=== GENERATING FIGURE: Benchmark I (Holonomic) ===\n');
  
  % Run benchmark simulations to get trajectory data
  res = benchmark_1_holonomic();
  
  env = res.env;
  P = res.P;
  traj_fuzzy = res.traj_fuzzy;
  traj_khatib = res.traj_khatib;
  n_fuzzy = res.n_fuzzy;
  n_khatib = res.n_khatib;
  conv_fuzzy = res.conv_fuzzy;  % Convergence flags for Fuzzy-APF
  conv_khatib = res.conv_khatib;  % Convergence flags for Khatib
  
  % Create figure with two panels
  fig = figure('Position', [100, 100, 1400, 600]);
  
  % ===== LEFT PANEL: Fuzzy-APF Trajectories =====
  ax1 = subplot(1, 2, 1);
  hold on; axis equal;
  
  % Draw obstacles (circles) with d0 interaction range (dashed)
  for i = 1:size(env.obs, 1)
    obs_x = env.obs(i, 1);
    obs_y = env.obs(i, 2);
    obs_r = env.obs(i, 3);
    % Get d0 value (from 4th column if available)
    if (size(env.obs, 2) >= 4)
      d0_val = env.obs(i, 4);
    else
      d0_val = P.d0;
    end
    theta_circ = linspace(0, 2*pi, 32);
    % Obstacle body (solid gray)
    patch(obs_x + obs_r * cos(theta_circ), obs_y + obs_r * sin(theta_circ), ...
          [0.7, 0.7, 0.7], 'EdgeColor', 'k', 'LineWidth', 0.5);
    % d0 interaction range (dashed red circle)
    plot(obs_x + d0_val * cos(theta_circ), obs_y + d0_val * sin(theta_circ), ...
         'r--', 'LineWidth', 1.0);
  end
  
  % Plot trajectories (Fuzzy-APF) with rainbow color gradient and rotated robot squares
  cmap = hsv(numel(traj_fuzzy));  % Rainbow colormap
  for i = 1:numel(traj_fuzzy)
    traj = traj_fuzzy{i};
    if (~isempty(traj) && size(traj, 1) > 1)
      % Rainbow gradient color
      plot(traj(:,1), traj(:,2), '-', 'Color', cmap(i, :), 'LineWidth', 1.5);
      
      % Add rotated robot squares with heading arrows along trajectory
      arrow_spacing = max(5, round(size(traj, 1) / 4));  % ~4 robots per trajectory
      for k = 1:arrow_spacing:size(traj, 1)-1
        % Heading direction from current to next point
        dx = traj(k+1, 1) - traj(k, 1);
        dy = traj(k+1, 2) - traj(k, 2);
        norm_d = sqrt(dx^2 + dy^2) + 1e-6;
        heading_angle = atan2(dy, dx);  % Heading angle in radians
        
        % Draw ROTATED solid black square robot
        robot_size = 0.3;
        robot_x = traj(k, 1);
        robot_y = traj(k, 2);
        % Square corners before rotation
        corners_x = robot_size * [-0.5, 0.5, 0.5, -0.5];
        corners_y = robot_size * [-0.5, -0.5, 0.5, 0.5];
        % Rotate corners by heading angle
        cos_h = cos(heading_angle);
        sin_h = sin(heading_angle);
        rotated_x = corners_x * cos_h - corners_y * sin_h + robot_x;
        rotated_y = corners_x * sin_h + corners_y * cos_h + robot_y;
        patch(rotated_x, rotated_y, 'k');
        
        % Draw arrow in black
        quiver(robot_x, robot_y, dx/norm_d*0.5, dy/norm_d*0.5, 0, ...
               'Color', 'k', 'LineWidth', 1.2, 'MaxHeadSize', 0.4);
      end
      
      % Starting position marker
      plot(traj(1,1), traj(1,2), 'o', 'Color', cmap(i, :), 'MarkerSize', 5, 'MarkerFaceColor', cmap(i, :));
      
      % If converged, draw robot at goal with color-coded heading
      if (conv_fuzzy(i) && size(traj, 1) > 1)
        final_pos = traj(end, :);
        % Heading angle from second-to-last to last point
        if (size(traj, 1) > 1)
          dx = traj(end, 1) - traj(end-1, 1);
          dy = traj(end, 2) - traj(end-1, 2);
        else
          dx = 0; dy = 0;
        end
        heading_angle = atan2(dy, dx);
        
        % Draw GREEN rotated square robot at goal
        robot_size = 0.4;
        corners_x = robot_size * [-0.5, 0.5, 0.5, -0.5];
        corners_y = robot_size * [-0.5, -0.5, 0.5, 0.5];
        cos_h = cos(heading_angle);
        sin_h = sin(heading_angle);
        rotated_x = corners_x * cos_h - corners_y * sin_h + final_pos(1);
        rotated_y = corners_x * sin_h + corners_y * cos_h + final_pos(2);
        patch(rotated_x, rotated_y, [0, 0.8, 0], 'EdgeColor', [0, 0.5, 0], 'LineWidth', 1.5);
      end
    end
  end
  
  % Goal marker (green star)
  plot(env.goal(1), env.goal(2), '*', 'Color', [0, 0.7, 0], 'MarkerSize', 20, 'LineWidth', 2);
  
  set(gca, 'XLim', [-10, 10], 'YLim', [-10, 10]);
  set(gca, 'XTick', [-10:5:10], 'YTick', [-10:5:10]);
  grid on; axis equal;
  xlabel('X (m)', 'FontSize', 11);
  ylabel('Y (m)', 'FontSize', 11);
  
  % ===== RIGHT PANEL: Khatib APF Trajectories =====
  ax2 = subplot(1, 2, 2);
  hold on; axis equal;
  
  % Draw obstacles (circles) with d0 interaction range (dashed)
  for i = 1:size(env.obs, 1)
    obs_x = env.obs(i, 1);
    obs_y = env.obs(i, 2);
    obs_r = env.obs(i, 3);
    % Get d0 value (from 4th column if available)
    if (size(env.obs, 2) >= 4)
      d0_val = env.obs(i, 4);
    else
      d0_val = P.d0;
    end
    theta_circ = linspace(0, 2*pi, 32);
    % Obstacle body (solid gray)
    patch(obs_x + obs_r * cos(theta_circ), obs_y + obs_r * sin(theta_circ), ...
          [0.7, 0.7, 0.7], 'EdgeColor', 'k', 'LineWidth', 0.5);
    % d0 interaction range (dashed red circle)
    plot(obs_x + d0_val * cos(theta_circ), obs_y + d0_val * sin(theta_circ), ...
         'r--', 'LineWidth', 1.0);
  end
  
  % Plot trajectories (Khatib) with rainbow color gradient and rotated robot squares
  cmap = hsv(numel(traj_khatib));  % Rainbow colormap
  for i = 1:numel(traj_khatib)
    traj = traj_khatib{i};
    if (~isempty(traj) && size(traj, 1) > 1)
      % Rainbow gradient color
      plot(traj(:,1), traj(:,2), '-', 'Color', cmap(i, :), 'LineWidth', 1.5);
      
      % Add rotated robot squares with heading arrows along trajectory
      arrow_spacing = max(5, round(size(traj, 1) / 4));  % ~4 robots per trajectory
      for k = 1:arrow_spacing:size(traj, 1)-1
        % Heading direction from current to next point
        dx = traj(k+1, 1) - traj(k, 1);
        dy = traj(k+1, 2) - traj(k, 2);
        norm_d = sqrt(dx^2 + dy^2) + 1e-6;
        heading_angle = atan2(dy, dx);  % Heading angle in radians
        
        % Draw ROTATED solid black square robot
        robot_size = 0.3;
        robot_x = traj(k, 1);
        robot_y = traj(k, 2);
        % Square corners before rotation
        corners_x = robot_size * [-0.5, 0.5, 0.5, -0.5];
        corners_y = robot_size * [-0.5, -0.5, 0.5, 0.5];
        % Rotate corners by heading angle
        cos_h = cos(heading_angle);
        sin_h = sin(heading_angle);
        rotated_x = corners_x * cos_h - corners_y * sin_h + robot_x;
        rotated_y = corners_x * sin_h + corners_y * cos_h + robot_y;
        patch(rotated_x, rotated_y, 'k');
        
        % Draw arrow in black
        quiver(robot_x, robot_y, dx/norm_d*0.5, dy/norm_d*0.5, 0, ...
               'Color', 'k', 'LineWidth', 1.2, 'MaxHeadSize', 0.4);
      end
      
      % Starting position marker
      plot(traj(1,1), traj(1,2), 'o', 'Color', cmap(i, :), 'MarkerSize', 5, 'MarkerFaceColor', cmap(i, :));
      
      % If converged, draw robot at goal with color-coded heading
      if (conv_khatib(i) && size(traj, 1) > 1)
        final_pos = traj(end, :);
        % Heading angle from second-to-last to last point
        if (size(traj, 1) > 1)
          dx = traj(end, 1) - traj(end-1, 1);
          dy = traj(end, 2) - traj(end-1, 2);
        else
          dx = 0; dy = 0;
        end
        heading_angle = atan2(dy, dx);
        
        % Draw GREEN rotated square robot at goal
        robot_size = 0.4;
        corners_x = robot_size * [-0.5, 0.5, 0.5, -0.5];
        corners_y = robot_size * [-0.5, -0.5, 0.5, 0.5];
        cos_h = cos(heading_angle);
        sin_h = sin(heading_angle);
        rotated_x = corners_x * cos_h - corners_y * sin_h + final_pos(1);
        rotated_y = corners_x * sin_h + corners_y * cos_h + final_pos(2);
        patch(rotated_x, rotated_y, [0, 0.8, 0], 'EdgeColor', [0, 0.5, 0], 'LineWidth', 1.5);
      end
    end
  end
  
  % Goal marker (green star)
  plot(env.goal(1), env.goal(2), '*', 'Color', [0, 0.7, 0], 'MarkerSize', 20, 'LineWidth', 2);
  
  set(gca, 'XLim', [-10, 10], 'YLim', [-10, 10]);
  set(gca, 'XTick', [-10:5:10], 'YTick', [-10:5:10]);
  grid on; axis equal;
  xlabel('X (m)', 'FontSize', 11);
  ylabel('Y (m)', 'FontSize', 11);
  
  % Save figure
  drawnow;
  print(gcf, 'comparison_fuzzy_vs_khatib.png', '-dpng', '-r150');
  print(gcf, 'comparison_fuzzy_vs_khatib.pdf', '-dpdf');
  fprintf('✓ Saved: /Users/adhaimc/Documents/GitHub/Fuzzy_APF26A/comparison_fuzzy_vs_khatib.png\n');
  fprintf('✓ Saved: /Users/adhaimc/Documents/GitHub/Fuzzy_APF26A/comparison_fuzzy_vs_khatib.pdf\n');
  
  close(fig);
end
