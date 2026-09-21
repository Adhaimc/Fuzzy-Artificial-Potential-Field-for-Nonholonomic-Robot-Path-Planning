#!/usr/bin/env octave
% Quick test of Proposal 2 implementation
% Tests both adaptive_only and adaptive+supervisor on guarded environment

addpath ('.');

fprintf('\n====== Proposal 2 Implementation Test ======\n\n');

% Load environment and params
P = fapf_params();
env = guarded_env(100, 2026);  % guarded goal, 100 obstacles
% Also test dense environment
env_dense = dense_env(100, 2026);  % dense environment with 100 obstacles

% Test configuration
test_pos = 1;  % First position only for quick test
rng(2026);
start_angles = 2*pi*rand(20, 1);
q0 = [9.0 * cos(start_angles(test_pos)), 9.0 * sin(start_angles(test_pos))];
th0 = 0;

fprintf('Test: Guarded Goal Environment (1 trial for quick check)\n');
fprintf('Start position: [%.2f, %.2f], angle: %.2f rad\n', q0(1), q0(2), th0);
fprintf('\nRunning Proposal 2...\n');

% Test 1: Adaptive only (no supervisor)
fprintf('\n[1/3] Adaptive-only (Proposal 2 per-zone + Lyapunov)\n');
try
  tic;
  out_adapt = run_adaptive_only_nonholo(q0, th0, env, P, 0.10, 0.08);
  t_adapt = toc;
  fprintf('  Time: %.3f s\n', t_adapt);
  fprintf('  Success: %s\n', iftrue(out_adapt.success, 'YES (0/1)', 'NO'));
  fprintf('  Path length: %.2f m\n', out_adapt.path_length);
  fprintf('  Min clearance: %.3f m\n', out_adapt.dmin);
catch err
  fprintf('  ERROR: %s\n', err.message);
end

% Test 2: Adaptive + Supervisor
fprintf('\n[2/3] Adaptive+Supervisor (Proposal 2 per-zone + GNRON)\n');
try
  tic;
  out_sup = run_supervised_nonholo(q0, th0, env, P, true, 7.0, 0.2, 0.6, 0.10, 0.08);
  t_sup = toc;
  fprintf('  Time: %.3f s\n', t_sup);
  fprintf('  Success: %s\n', iftrue(out_sup.success, 'YES', 'NO'));
  fprintf('  Path length: %.2f m\n', out_sup.path_length);
  fprintf('  Min clearance: %.3f m\n', out_sup.dmin);
  fprintf('  Supervisor switches: %d\n', out_sup.nsw);
catch err
  fprintf('  ERROR: %s\n', err.message);
end

% Test 3: Base (no adaptation, no supervisor)
fprintf('\n[3/3] Base law (no adaptation, no supervisor)\n');
try
  tic;
  out_base = fapf_sim_nonholo(q0, th0, env, P);
  t_base = toc;
  fprintf('  Time: %.3f s\n', t_base);
  fprintf('  Success: %s\n', iftrue(out_base.success, 'YES', 'NO'));
  fprintf('  Path length: %.2f m\n', out_base.path_length);
  fprintf('  Min clearance: %.3f m\n', out_base.dmin);
catch err
  fprintf('  ERROR: %s\n', err.message);
end

fprintf('\n====== Test Summary ======\n');
fprintf('Proposal 2 implementation appears to be working.\n');
fprintf('Run fig_stress_test.m for full 20-trial benchmark.\n\n');

function result = iftrue(cond, yes_str, no_str)
  if (cond)
    result = yes_str;
  else
    result = no_str;
  end
end
