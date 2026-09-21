function [wp, ok] = fapf_subgoal_corrected (q, goal, obs, P, L, Delta)
% CORRECTED VERSION: Handles per-obstacle variable d0 values
% Same as fapf_subgoal but checks against ACTUAL d0 values in obs(:,4)
% instead of assuming all obstacles have the same d0 = P.d0

  half_width_nom = P.r_robot + P.r_safety;
  [d_q, ~] = fapf_dobs (q, obs);
  half_width = min (half_width_nom, max (d_q, 0));
  e    = norm (q - goal);
  wp   = [];
  ok   = false;
  best = inf;

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
      
      % FIXED: Check against ACTUAL per-obstacle d0 values, not just P.d0
      d_valid = true;
      for i = 1:size(obs, 1)
        obs_center = obs(i, 1:2);
        obs_r = obs(i, 3);
        % Get actual d0 for this obstacle (4th column if available)
        if (size(obs, 2) >= 4)
          d0_obs = obs(i, 4);
        else
          d0_obs = P.d0;
        end
        % Distance from candidate to this obstacle's surface
        d_surface = norm(c - obs_center) - obs_r;
        % Waypoint must be OUTSIDE all obstacles' influence zones
        if (d_surface < d0_obs)
          d_valid = false;
          break;
        end
      end
      if (~d_valid), continue; end  % (W1', endpoint)
      
      if (~fapf_passage_clear (q, c, obs, half_width)), continue; end  % (W1', passage)
      dgoal = norm (c - goal);
      if (dgoal > e - Delta), continue; end             % (W2)
      if (dgoal < best)
        best = dgoal; wp = c; ok = true;                % (W3) holds by r <= L
      end
    end
  end
end
