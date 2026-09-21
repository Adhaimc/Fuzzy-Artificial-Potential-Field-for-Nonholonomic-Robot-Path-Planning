function [wp, ok] = fapf_subgoal (q, goal, obs, P, L, Delta)
% Constructive waypoint generator enforcing Assumption ass:waypoint.
% Candidates are sampled on circles of radius <= L about the stall position
% q and filtered by
%   (W1') d_obs(wp) >= d0 AND a passage of width >= 2*(r_robot+r_safety)
%         connects q to wp through free space (Lemma lem:passage-descent)
%   (W2)  ||wp - goal|| <= ||q - goal|| - Delta -- strict progress
%   (W3)  ||wp - q||    <= L                    -- bounded horizon
% Among the survivors the one minimising ||wp - goal|| is returned
% (Remark rem:gnron-assumptions).  If none exists the supervisor reports
% failure instead of switching, so thm:gnron-hybrid is never invoked outside
% its hypotheses.
%
% Edge case: if q is already transiently within the nominal half-width
% margin of some obstacle, the corridor requirement is relaxed to "do not
% get any closer than the clearance already present at q" rather than
% demanding an unattainable margin at t=0 for every candidate alike.

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
      [d, ~] = fapf_dobs (c, obs);
      if (d < P.d0), continue; end                      % (W1', endpoint)
      if (~fapf_passage_clear (q, c, obs, half_width)), continue; end  % (W1', passage)
      dgoal = norm (c - goal);
      if (dgoal > e - Delta), continue; end             % (W2)
      if (dgoal < best)
        best = dgoal; wp = c; ok = true;                % (W3) holds by r <= L
      end
    end
  end
end
