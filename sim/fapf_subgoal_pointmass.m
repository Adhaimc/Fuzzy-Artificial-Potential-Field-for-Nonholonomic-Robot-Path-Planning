function [wp, ok] = fapf_subgoal_pointmass(q, goal, obs_centers, P, L, Delta)
% Constructive waypoint generator using POINT-MASS obstacle model.
%
% Same logic as fapf_subgoal but with point-mass distances.
%
% Criteria:
%   (W1') d_obs(wp) >= d0  (endpoint clearance, point-mass model)
%   (W2)  ||wp - goal|| <= ||q - goal|| - Delta
%   (W3)  ||wp - q||    <= L

  e    = norm(q - goal);
  wp   = [];
  ok   = false;
  best = inf;

  % the true goal is itself an admissible waypoint once it is within horizon
  d_goal_pm = fapf_dobs_pointmass(goal, obs_centers);
  if (e <= L && d_goal_pm >= P.d0)
    wp = goal(:).'; ok = true; return;
  end

  angs = linspace(0, 2*pi, 73); angs(end) = [];
  rads = L * [1.0, 0.8, 0.6, 0.4];

  for a = angs
    u = [cos(a), sin(a)];
    for r = rads
      c = q(:).' + r * u;
      if (norm(c - goal) > P.D_max), continue; end
      
      d = fapf_dobs_pointmass(c, obs_centers);
      if (d < P.d0), continue; end                      % (W1')
      
      dgoal = norm(c - goal);
      if (dgoal > e - Delta), continue; end             % (W2)
      
      if (dgoal < best)
        best = dgoal; wp = c; ok = true;                % (W3) holds by r <= L
      end
    end
  end
end
