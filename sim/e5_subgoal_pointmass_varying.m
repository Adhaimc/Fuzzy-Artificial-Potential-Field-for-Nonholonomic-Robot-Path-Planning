function res = e5_subgoal_pointmass_varying()
% E5-VARYING: Test GNRON with point-mass obstacles having VARYING d0 per obstacle.
%
% Compares three models:
%   (A) Circular obstacles (baseline)     -> 18/20
%   (B) Point-mass uniform d0=0.2         -> 20/20
%   (C) Point-mass varying d0 ∈ [0.1,1]  -> ? (richer, heterogeneous obstacles)
%
% Hypothesis: varying d0 should still achieve 20/20 (or close to it) because
% the theory still holds—each obstacle just has a different influence region.

  P = fapf_params();
  P.k_a = 9.6;
  P.k_r = 16.0;
  P.v_max = 2.0;
  
  fprintf('\n=== E5-VARYING: Circular vs. Uniform vs. Varying d0 Models ===\n');
  fprintf('Gains: k_a=%.1f, k_r=%.1f, v_max=%.1f\n', P.k_a, P.k_r, P.v_max);
  fprintf('Params: d0_nominal=%.1f, r_safety=%.1f\n\n', P.d0, P.r_safety);
  
  % Create environments
  env_circular = fapf_env(100, 2026, P, 'random');
  obs_centers = env_circular.obs(:, 1:2);
  obs_uniform_centers = obs_centers;  % Just centers for uniform d0
  obs_varying = create_varying_d0_struct(obs_centers);  % Struct with per-obstacle d0
  
  nstart = 20;
  results_circular = [];
  results_uniform = [];
  results_varying = [];
  
  fprintf('%-5s %-12s %-12s %-12s\n', 'Traj', 'Circular', 'Uniform', 'Varying');
  fprintf('%-5s %-12s %-12s %-12s\n', '----', '--------', '--------', '--------');
  
  for i = 1:nstart
    angle = 2*pi * (i-1) / nstart;
    q0 = env_circular.goal + 0.9 * P.D_max * [cos(angle), sin(angle)];
    
    conv_circ = run_traj(q0, env_circular, P, env_circular.obs, 'circular');
    conv_unif = run_traj(q0, env_circular, P, obs_uniform_centers, 'pointmass_uniform');
    conv_vary = run_traj(q0, env_circular, P, obs_varying, 'pointmass_varying');
    
    if (conv_circ), circ_str = 'CONV'; else, circ_str = 'FAIL'; end
    if (conv_unif), unif_str = 'CONV'; else, unif_str = 'FAIL'; end
    if (conv_vary), vary_str = 'CONV'; else, vary_str = 'FAIL'; end
    fprintf('%-5d %-12s %-12s %-12s\n', i, circ_str, unif_str, vary_str);
    
    results_circular = [results_circular, conv_circ];
    results_uniform = [results_uniform, conv_unif];
    results_varying = [results_varying, conv_vary];
  end
  
  n_circ = sum(results_circular);
  n_unif = sum(results_uniform);
  n_vary = sum(results_varying);
  fprintf('\n%-30s %d/20 (%.0f%%)\n', 'Circular obstacles:', n_circ, 100*n_circ/20);
  fprintf('%-30s %d/20 (%.0f%%)\n', 'Point-mass uniform d0:', n_unif, 100*n_unif/20);
  fprintf('%-30s %d/20 (%.0f%%)\n', 'Point-mass varying d0:', n_vary, 100*n_vary/20);
  
  res = struct('circ', n_circ, 'uniform', n_unif, 'varying', n_vary);
end

%% Helper: Create point-mass obstacles with VARYING d0 ∈ [0.2, 0.4], mean 0.3
function obs_struct = create_varying_d0_struct(obs_centers)
  nobs = size(obs_centers, 1);
  % Generate d0 values uniformly distributed in [0.2, 0.4] with mean 0.3
  rng(2026);  % reproducible
  d0_vals = 0.2 + 0.2*rand(nobs, 1);  % Uniform in [0.2, 0.4]
  
  obs_struct = struct('center', {}, 'd0', {});
  for i = 1:nobs
    obs_struct(i).center = obs_centers(i, :);
    obs_struct(i).d0 = d0_vals(i);
  end
end

%% Helper: Run a single trajectory
function converged = run_traj(q0, env, P, obs_input, model_type)
  q = q0(:).';
  converged = false;
  
  % Initial kick
  margin = P.r_robot + P.r_safety;
  for step_kick = 1:100
    if (strcmp(model_type, 'circular'))
      d_obs_kick = fapf_dobs(q, obs_input);
    elseif (strcmp(model_type, 'pointmass_uniform'))
      d_obs_kick = fapf_dobs_pointmass(q, obs_input);
    else  % pointmass_varying
      [d_obs_kick, ~, ~] = fapf_dobs_pointmass_varying(q, obs_input);
    end
    if (d_obs_kick >= margin), break; end
    
    F_escape = -(q - env.goal) / (norm(q - env.goal) + 1e-6);
    q = q + P.v_max * P.dt * F_escape / (norm(F_escape) + 1e-6);
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
      return;
    end
    
    % Stall detection
    if (strcmp(model_type, 'circular'))
      d_obs = fapf_dobs(q, obs_input);
    elseif (strcmp(model_type, 'pointmass_uniform'))
      d_obs = fapf_dobs_pointmass(q, obs_input);
    else  % pointmass_varying
      [d_obs, ~, ~] = fapf_dobs_pointmass_varying(q, obs_input);
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
        elseif (strcmp(model_type, 'pointmass_uniform'))
          [wp, wp_ok] = fapf_subgoal_pointmass(q, env.goal, obs_input, P, L, 0.5);
        else  % pointmass_varying
          [wp, wp_ok] = fapf_subgoal_pointmass_varying(q, env.goal, obs_input, P, L, 0.5);
        end
        if (~wp_ok), return; end
        q = wp(:).';
        V_history = [];
      end
    end
    
    % Force law
    if (strcmp(model_type, 'circular'))
      [F, ~] = fapf_force(q, env.goal, obs_input, P);
    elseif (strcmp(model_type, 'pointmass_uniform'))
      [F, ~] = fapf_force_pointmass(q, env.goal, obs_input, P);
    else  % pointmass_varying
      [F, ~] = fapf_force_pointmass_varying(q, env.goal, obs_input, P);
    end
    q = q + (P.v_max / (norm(F) + P.eps_v)) * F * P.dt;
  end
end
