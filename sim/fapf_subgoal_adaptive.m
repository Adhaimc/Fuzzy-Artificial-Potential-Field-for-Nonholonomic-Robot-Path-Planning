function [wp, ok] = fapf_subgoal_adaptive (q, goal, obs, P, L)
% Adaptive waypoint generator: relaxes progress requirement Δ based on
% clearance and proximity to goal.
%
% Replaces the fixed Δ_nom with:
%   Δ(d_goal, d_obs) = Δ_nom · w_clearance(d_obs) · w_goal(d_goal)
%
% where:
%   w_clearance(d_obs) = exp(-(d_obs - d0)² / σ²)
%     When trapped near obstacles (d_obs ≈ d0): w_clearance ≈ 1 → strict Δ
%     When in open space (d_obs ≫ d0):         w_clearance ≈ 0 → relax Δ → any forward
%
%   w_goal(d_goal) = tanh(d_goal / d_threshold)
%     When far from goal:  w_goal ≈ 1 → demand full progress
%     When near goal:      w_goal ≈ 0 → relax Δ → let attraction take over
%
% Candidates are sampled on circles of radius <= L about stall position q
% and filtered by (W1'), (W2-adaptive), (W3) as in fapf_subgoal.m.
%
% Edge case (dynamic corridor relaxation): same as fapf_subgoal.m

  half_width_nom = P.r_robot + P.r_safety;
  [d_q, ~] = fapf_dobs (q, obs);
  half_width = min (half_width_nom, max (d_q, 0));
  e    = norm (q - goal);
  wp   = [];
  ok   = false;
  best = inf;

  % Compute adaptive progress threshold
  w_clear = exp(-((d_q - P.d0) / P.sigma_clear)^2);
  w_goal  = tanh(e / P.d_threshold_goal);
  Delta_adaptive = P.Delta_nom * w_clear * w_goal;

  % the true goal is itself an admissible waypoint once it is within horizon
  [dg, ~] = fapf_dobs (goal, obs);
  if (e <= L && dg >= P.d0 && fapf_passage_clear (q, goal, obs, half_width))
    wp = goal(:).'; ok = true; return;
  end

  angs = linspace (0, 2*pi, 73); angs(end) = [];
  rads = L * [1.0, 0.8, 0.6, 0.4];

  for a = angs
    u = [cos(a), sin(a)];
    for r = rads
      c = q(:).' + r * u;
      if (norm (c - goal) > P.D_max), continue; end     % stay in workspace
      [d, ~] = fapf_dobs (c, obs);
      if (d < P.d0), continue; end                      % (W1', endpoint)
      if (~fapf_passage_clear (q, c, obs, half_width)), continue; end  % (W1', passage)
      dgoal = norm (c - goal);
      if (dgoal > e - Delta_adaptive), continue; end    % (W2-adaptive)
      if (dgoal < best)
        best = dgoal; wp = c; ok = true;                % (W3) holds by r <= L
      end
    end
  end
end
