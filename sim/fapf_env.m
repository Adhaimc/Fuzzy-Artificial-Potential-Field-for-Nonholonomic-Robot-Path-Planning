function env = fapf_env (nobs, seed, P, mode)
% Random obstacle field in a disc of radius P.D_max centred on the goal.
% Goal clearance (hypothesis 3 of Theorem thm:no-local-minima-fuzzy) is
% enforced: every obstacle satisfies ||goal - o_i|| >= d0 + eps_clear.

  if (nargin < 4), mode = 'random'; end

  rand ('state', seed);
  env.goal = [0, 0];
  env.eps_clear = 0.5;
  r_obs = 0.30;

  switch (mode)
    case 'random'
      obs = zeros (nobs, 3);
      k = 0; guard = 0;
      while (k < nobs && guard < 200000)
        guard++;
        ang = 2*pi*rand ();
        rad = P.D_max * sqrt (rand ());
        c = [rad*cos(ang), rad*sin(ang)];
        if (norm (c - env.goal) < P.d0 + env.eps_clear + r_obs), continue; end
        if (k > 0)
          dd = sqrt (sum ((obs(1:k,1:2) - c).^2, 2));
          if (min (dd) < 2.2*r_obs), continue; end
        end
        k++;
        obs(k,:) = [c, r_obs];
      end
      env.obs = obs(1:k,:);

    case 'symmetric'
      % Adversarial collinear/symmetric pair straddling the goal axis:
      % the configuration in which Remark rem:theorem3-necessity states
      % that a perpendicular perturbation cannot generate escape.
      g = 1.15;
      env.obs = [ -4.0,  g, r_obs;
                  -4.0, -g, r_obs ];

    case 'voronoi'
      % Two equidistant obstacles: the robot must slide along the Voronoi
      % boundary (Remark rem:filippov).
      env.obs = [ -4.0,  0.75, r_obs;
                  -4.0, -0.75, r_obs ];
  end
end
