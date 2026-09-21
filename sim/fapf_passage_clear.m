function ok = fapf_passage_clear (q1, q2, obs, half_width)
% Passage feasibility check (Lemma lem:passage-descent / Assumption W1').
% Mirrors matlab/updatedlaw_passage_clear.m. Returns true iff the
% straight-line corridor from q1 to q2 keeps clearance >= half_width from
% every obstacle surface along its entire length, i.e. a passage of width
% >= 2*half_width = 2*(r_robot + r_safety) connects the two points.
  ok = true;
  q1 = q1(:).'; q2 = q2(:).';
  ab = q2 - q1;
  denom = dot (ab, ab);
  for i = 1:size(obs, 1)
    c = obs(i, 1:2);
    r = obs(i, 3);
    if (denom < 1e-12)
      t = 0;
    else
      t = dot (c - q1, ab) / denom;
      t = max (0, min (1, t));
    end
    proj = q1 + t * ab;
    clearance = norm (c - proj) - r;
    if (clearance < half_width)
      ok = false;
      return;
    end
  end
end
