function res = e5_subgoal_adaptive()
% E5-ADAPTIVE: Compare fixed vs. adaptive subgoal on 100-obstacle benchmark.
%
% Tests whether adaptive Δ(d_goal, d_obs) improves convergence on the dense
% benchmark where 2-3 failing trajectories get stuck outside the annulus.

  % Setup: dense 100-obstacle environment
  P = fapf_params();
  P.k_a = 9.6;    % 8x manuscript default
  P.k_r = 16.0;   % 8x manuscript default
  P.v_max = 2.0;  % 2x manuscript default
  
  % Adaptive parameters
  P.Delta_nom = 0.5;
  P.sigma_clear = 0.3;
  P.d_threshold_goal = 2.0;
  
  fprintf('\n=== E5-ADAPTIVE: Comparing Fixed vs. Adaptive Waypoint Subgoal ===\n');
  fprintf('Gains: k_a=%.1f, k_r=%.1f, v_max=%.1f\n', P.k_a, P.k_r, P.v_max);
  fprintf('Params: d0=%.1f, r_safety=%.1f, r_robot=%.1f\n', P.d0, P.r_safety, P.r_robot);
  fprintf('Adaptive: Δ_nom=%.1f, σ_clear=%.1f, d_threshold=%.1f\n\n', ...
          P.Delta_nom, P.sigma_clear, P.d_threshold_goal);
  
  % Create environment once for consistency
  env = fapf_env(100, 2026, P, 'random');
  
  % Generate 20 start positions (fixed annulus positions)
  nstart = 20;
  results_fixed = [];
  results_adaptive = [];
  
  fprintf('%-5s %-12s %-12s %-12s %-12s\n', 'Traj', 'Fixed', 'Adaptive', 'd_goal(fixed)', 'd_goal(adapt)');
  fprintf('%-5s %-12s %-12s %-12s %-12s\n', '----', '--------', '--------', '----------', '----------');
  
  for i = 1:nstart
    % Fixed start position on 9m annulus
    angle = 2*pi * (i-1) / nstart;
    q0 = env.goal + 0.9 * P.D_max * [cos(angle), sin(angle)];
    
    % Run with fixed Delta
    [converged_fixed, d_goal_fixed] = run_traj(q0, env, P, 'fixed');
    
    % Run with adaptive Delta
    [converged_adapt, d_goal_adapt] = run_traj(q0, env, P, 'adaptive');
    
    % Display
    if (converged_fixed)
      fixed_str = 'CONV';
    else
      fixed_str = 'FAIL';
    end
    if (converged_adapt)
      adapt_str = 'CONV';
    else
      adapt_str = 'FAIL';
    end
    fprintf('%-5d %-12s %-12s %-12.3f %-12.3f\n', i, fixed_str, adapt_str, d_goal_fixed, d_goal_adapt);
    
    results_fixed = [results_fixed, converged_fixed];
    results_adaptive = [results_adaptive, converged_adapt];
  end
  
  % Summary
  nconv_fixed = sum(results_fixed);
  nconv_adapt = sum(results_adaptive);
  fprintf('\n%-30s %d/20 (%.0f%%)\n', 'Fixed Delta:', nconv_fixed, 100*nconv_fixed/20);
  fprintf('%-30s %d/20 (%.0f%%)\n', 'Adaptive Delta:', nconv_adapt, 100*nconv_adapt/20);
  
  if (nconv_adapt > nconv_fixed)
    fprintf('\n✓ IMPROVEMENT: Adaptive Δ gained %d trajectory(ies)!\n', nconv_adapt - nconv_fixed);
  elseif (nconv_adapt == nconv_fixed)
    fprintf('\n= Same: Both achieved %d/20 convergence.\n', nconv_fixed);
  else
    fprintf('\n✗ REGRESSION: Adaptive Δ lost %d trajectory(ies).\n', nconv_fixed - nconv_adapt);
  end
  
  res = struct('nconv_fixed', nconv_fixed, 'nconv_adaptive', nconv_adapt);
end

%% Helper: Run a single trajectory
function [converged, d_goal_end] = run_traj(q0, env, P, mode)
  q = q0(:).';
  converged = false;
  d_goal_end = norm(q - env.goal);
  
  % Initial kick
  margin = P.r_robot + P.r_safety;
  steps_kick = 0;
  while (fapf_dobs(q, env.obs) < margin && steps_kick < 100)
    F_escape = -(q - env.goal) / (norm(q - env.goal) + 1e-6);
    q = q + P.v_max * P.dt * F_escape / (norm(F_escape) + 1e-6);
    steps_kick = steps_kick + 1;
  end
  
  % Main loop
  max_steps = 12000;
  V_history = [];
  tau_dwell = 1.0;
  T_window = 0.5;
  eps_conv = 1e-6;
  L = 3.0;
  
  for step = 1:max_steps
    d_goal = norm(q - env.goal);
    if (d_goal <= P.rho)
      converged = true;
      d_goal_end = d_goal;
      return;
    end
    
    % Stall detection
    d_obs = fapf_dobs(q, env.obs);
    V = d_goal^2 + max(0, P.d0 - d_obs);
    V_history = [V_history, V];
    
    t_steps = round(T_window / P.dt);
    if (numel(V_history) > t_steps)
      V_history(1) = [];
    end
    
    if (numel(V_history) >= t_steps)
      Vdot = (V_history(end) - V_history(1)) / T_window;
      if (abs(Vdot) < eps_conv && step > round(tau_dwell / P.dt))
        % Request waypoint
        if (strcmp(mode, 'fixed'))
          [wp, wp_ok] = fapf_subgoal(q, env.goal, env.obs, P, L, 0.5);
        else
          [wp, wp_ok] = fapf_subgoal_adaptive(q, env.goal, env.obs, P, L);
        end
        if (~wp_ok)
          d_goal_end = norm(q - env.goal);
          return;
        end
        q = wp(:).';
        V_history = [];
      end
    end
    
    % Force law
    [F, ~] = fapf_force(q, env.goal, env.obs, P);
    q = q + (P.v_max / (norm(F) + P.eps_v)) * F * P.dt;
  end
  
  d_goal_end = norm(q - env.goal);
end
