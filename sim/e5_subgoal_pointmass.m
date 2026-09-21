function res = e5_subgoal_pointmass()
% E5-POINTMASS: Test GNRON convergence with POINT-MASS obstacles (theoretical model).
%
% Hypothesis: The 2/20 failures are due to theory-practice gap (circular vs. point obstacles).
% If we use the theoretical point-mass model, Theorem 2 should guarantee 20/20 convergence.
%
% Compares:
%   (A) Circular obstacles (current sim model) -> 18/20
%   (B) Point-mass obstacles (theoretical model) -> expect 20/20

  P = fapf_params();
  P.k_a = 9.6;    % 8x manuscript default
  P.k_r = 16.0;   % 8x manuscript default
  P.v_max = 2.0;  % 2x manuscript default
  
  fprintf('\n=== E5-POINTMASS: Circular vs. Point-Mass Obstacle Models ===\n');
  fprintf('Gains: k_a=%.1f, k_r=%.1f, v_max=%.1f\n', P.k_a, P.k_r, P.v_max);
  fprintf('Params: d0=%.1f, r_safety=%.1f, r_robot=%.1f\n\n', P.d0, P.r_safety, P.r_robot);
  
  % Create environment once for consistency
  env = fapf_env(100, 2026, P, 'random');
  
  % Extract obstacle centers (for point-mass model)
  obs_centers = env.obs(:, 1:2);
  
  % Generate 20 start positions
  nstart = 20;
  results_circular = [];
  results_pointmass = [];
  
  fprintf('%-5s %-12s %-12s %-12s %-12s\n', 'Traj', 'Circular', 'PointMass', 'd_goal(circ)', 'd_goal(pm)');
  fprintf('%-5s %-12s %-12s %-12s %-12s\n', '----', '--------', '--------', '----------', '----------');
  
  for i = 1:nstart
    angle = 2*pi * (i-1) / nstart;
    q0 = env.goal + 0.9 * P.D_max * [cos(angle), sin(angle)];
    
    % Run with circular obstacles
    [conv_circ, d_goal_circ] = run_traj(q0, env, P, env.obs, 'circular');
    
    % Run with point-mass obstacles
    [conv_pm, d_goal_pm] = run_traj(q0, env, P, obs_centers, 'pointmass');
    
    % Display
    if (conv_circ)
      circ_str = 'CONV';
    else
      circ_str = 'FAIL';
    end
    if (conv_pm)
      pm_str = 'CONV';
    else
      pm_str = 'FAIL';
    end
    fprintf('%-5d %-12s %-12s %-12.3f %-12.3f\n', i, circ_str, pm_str, d_goal_circ, d_goal_pm);
    
    results_circular = [results_circular, conv_circ];
    results_pointmass = [results_pointmass, conv_pm];
  end
  
  % Summary
  nconv_circ = sum(results_circular);
  nconv_pm = sum(results_pointmass);
  fprintf('\n%-30s %d/20 (%.0f%%)\n', 'Circular obstacles:', nconv_circ, 100*nconv_circ/20);
  fprintf('%-30s %d/20 (%.0f%%)\n', 'Point-mass obstacles:', nconv_pm, 100*nconv_pm/20);
  
  if (nconv_pm > nconv_circ)
    fprintf('\n✓ THEORY VALIDATED: Point-mass model achieves %d/20!\n', nconv_pm);
  else
    fprintf('\n? No difference: both achieve %d/20.\n', nconv_circ);
  end
  
  res = struct('nconv_circular', nconv_circ, 'nconv_pointmass', nconv_pm);
end

%% Helper: Run a single trajectory
function [converged, d_goal_end] = run_traj(q0, env, P, obs_input, model_type)
  q = q0(:).';
  converged = false;
  d_goal_end = norm(q - env.goal);
  
  % Initial kick
  margin = P.r_robot + P.r_safety;
  steps_kick = 0;
  while (steps_kick < 100)
    if (strcmp(model_type, 'circular'))
      d_obs_kick = fapf_dobs(q, obs_input);
    else
      d_obs_kick = fapf_dobs_pointmass(q, obs_input);
    end
    if (d_obs_kick >= margin), break; end
    
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
    if (strcmp(model_type, 'circular'))
      d_obs = fapf_dobs(q, obs_input);
    else
      d_obs = fapf_dobs_pointmass(q, obs_input);
    end
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
        if (strcmp(model_type, 'circular'))
          [wp, wp_ok] = fapf_subgoal(q, env.goal, obs_input, P, L, 0.5);
        else
          [wp, wp_ok] = fapf_subgoal_pointmass(q, env.goal, obs_input, P, L, 0.5);
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
    if (strcmp(model_type, 'circular'))
      [F, ~] = fapf_force(q, env.goal, obs_input, P);
    else
      [F, ~] = fapf_force_pointmass(q, env.goal, obs_input, P);
    end
    q = q + (P.v_max / (norm(F) + P.eps_v)) * F * P.dt;
  end
  
  d_goal_end = norm(q - env.goal);
end
