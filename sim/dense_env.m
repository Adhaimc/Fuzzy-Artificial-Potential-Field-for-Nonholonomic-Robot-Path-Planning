function env = dense_env (nobs_total, seed)
% Dense random obstacle field WITHOUT guarded goal structure.
% Obstacles distributed uniformly, smaller minimum separation to create
% clutter. Used to test adaptive law in challenging scenarios.

  rand ('state', seed);
  goal = [0, 0];
  rw = 9;

  % Generate obstacles uniformly throughout workspace, no guarding ring
  obs = zeros (nobs_total, 3);
  k = 0; guard = 0;
  
  while (k < nobs_total && guard < 500000)
    guard++;
    ang = 2*pi*rand ();
    rad = rw * sqrt (rand ());
    c = [rad*cos(ang), rad*sin(ang)];
    r = (0.15 + 0.35*rand ()) / 2;  % Smaller obstacles, more clutter
    
    % Keep away from goal (but less aggressively than guarded_env)
    if (norm (c - goal) < 1.0), continue; end
    
    % Check separation from existing obstacles (tighter packing = denser)
    if (k > 0)
      dd = sqrt (sum ((obs(1:k,1:2) - c).^2, 2));
      if (min (dd) < 1.5*r + 0.2), continue; end  % Reduced from 2*r + 0.3
    end
    
    k++;
    obs(k,:) = [c, r];
  end

  env.goal = goal;
  env.obs  = obs(1:k,:);
end
