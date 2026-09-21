function [wp, ok] = fapf_subgoal_pointmass_varying(q, goal, obs_struct, P, L, Delta)
% Constructive waypoint generator with VARYING d0 per obstacle (point-mass model).
%
% Criteria:
%   (W1') d_i(wp) >= d0_i for ALL obstacles i
%   (W2)  ||wp - goal|| <= ||q - goal|| - Delta
%   (W3)  ||wp - q||    <= L

  e = norm(q - goal);
  wp = [];
  ok = false;
  best = inf;

  % Check if goal is admissible
  goal_ok = true;
  if (~isempty(obs_struct))
    for i = 1:numel(obs_struct)
      d_goal_i = norm(goal - obs_struct(i).center);
      if (d_goal_i < obs_struct(i).d0)
        goal_ok = false;
        break;
      end
    end
  end
  if (e <= L && goal_ok)
    wp = goal(:).'; ok = true; return;
  end

  angs = linspace(0, 2*pi, 73); angs(end) = [];
  rads = L * [1.0, 0.8, 0.6, 0.4];

  for a = angs
    u = [cos(a), sin(a)];
    for r = rads
      c = q(:).' + r * u;
      if (norm(c - goal) > P.D_max), continue; end
      
      % Check clearance against all obstacles
      candidate_ok = true;
      if (~isempty(obs_struct))
        for i = 1:numel(obs_struct)
          d_c_i = norm(c - obs_struct(i).center);
          if (d_c_i < obs_struct(i).d0)
            candidate_ok = false;
            break;
          end
        end
      end
      if (~candidate_ok), continue; end  % (W1')
      
      dgoal = norm(c - goal);
      if (dgoal > e - Delta), continue; end  % (W2)
      
      if (dgoal < best)
        best = dgoal; wp = c; ok = true;  % (W3) holds by r <= L
      end
    end
  end
end
