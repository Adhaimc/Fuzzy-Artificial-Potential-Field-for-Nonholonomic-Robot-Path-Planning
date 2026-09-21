function env = guarded_env (nobs_total, seed)
% Dense random field with a guarded goal: a ring of obstacles around the
% goal, closed except for a single gate near angle 0 (east). Used by
% fig_stress_test.m (fig:trajectories-100-obs).

  rand ('state', seed);
  goal = [0, 0];
  rw = 9;

  ring_r   = 1.3;
  r_ring   = 0.3;
  ring_ang = linspace (30*pi/180, 330*pi/180, 6);
  ring_obs = [ring_r*cos(ring_ang(:)), ring_r*sin(ring_ang(:)), r_ring*ones(6,1)];

  n_ring = size (ring_obs, 1);
  n_fill = nobs_total - n_ring;
  fill_obs = zeros (n_fill, 3);
  k = 0; guard = 0;
  while (k < n_fill && guard < 300000)
    guard++;
    ang = 2*pi*rand ();
    rad = rw * sqrt (rand ());
    c = [rad*cos(ang), rad*sin(ang)];
    r = (0.2 + 0.4*rand ()) / 2;
    if (norm (c - goal) < 2.3), continue; end
    if (any (sqrt (sum ((ring_obs(:,1:2) - c).^2, 2)) < (ring_obs(:,3) + r + 0.15))), continue; end
    if (k > 0)
      dd = sqrt (sum ((fill_obs(1:k,1:2) - c).^2, 2));
      if (min (dd) < 2*r + 0.3), continue; end
    end
    k++;
    fill_obs(k,:) = [c, r];
  end

  env.goal = goal;
  env.obs  = [ring_obs; fill_obs(1:k,:)];
end
