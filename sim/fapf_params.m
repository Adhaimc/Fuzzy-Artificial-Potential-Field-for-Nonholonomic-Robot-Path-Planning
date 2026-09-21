function P = fapf_params (varargin)
% Design constants for the corrected Fuzzy-APF formulation.
% All values correspond to the revised manuscript (Sections 3-5).
% Override any field with name/value pairs, e.g. fapf_params('k_a', 5.0).

  % --- gains -------------------------------------------------------------
  P.k_a       = 1.2;      % attractive gain
  P.k_r       = 2.0;      % repulsive gain

  % --- repulsive structure (Assumption ass:repulsive-structure) ----------
  P.d0        = 2.0;      % influence range  [m]
  P.r_safety  = 0.5;      % minimum safety distance [m]
  P.eps_reg   = 0.01;     % regularisation in implemented model
  P.r_robot   = 0.3;      % robot radius, for passage-width check (Lemma lem:passage-descent) [m]

  % --- fuzzy partition ---------------------------------------------------
  P.N         = 5;        % number of distance zones
  P.d_meas    = 3.0;      % sensor range spanned by the partition [m]
  P.w1_min    = 0.3;  P.w1_max = 1.0;
  P.w2_min    = 0.2;  P.w2_max = 1.0;

  % --- kinematics / termination -----------------------------------------
  P.rho       = 0.2;      % terminal ball radius = delta_goal [m]
  P.eps_v     = 0.10;     % velocity floor fraction
  P.v_max     = 1.0;      % max linear speed [m/s]
  P.k_omega   = 3.0;      % steering gain
  P.dt        = 1e-3;     % integration step [s]
  P.T_max     = 60.0;     % simulation horizon [s]

  % --- GNRON waypoint progress (adaptive scheme) -------------------------
  P.Delta_nom = 0.5;      % nominal progress threshold Δ_nom [m]
  P.sigma_clear = 0.3;    % clearance sensitivity width [m]
  P.d_threshold_goal = 2.0; % goal distance threshold for relaxation [m]

  % --- workspace ---------------------------------------------------------
  P.D_max     = 14.1;     % sup ||q-goal|| over the 10x10 m workspace [m]
                          % (matches the manuscript's design-window/CBF
                          % worked examples: alpha0 = 11.28 s^-1, safety
                          % bound k_a/k_r <= 1.36)

  % --- adaptation --------------------------------------------------------
  P.alpha_c   = 0.05;     % centre adaptation rate
  P.alpha_s   = 0.05;     % width adaptation rate
  P.vmax_c    = 0.20;     % rate limit on centres
  P.vmax_s    = 0.10;     % rate limit on widths

  % --- user overrides ----------------------------------------------------
  for i = 1:2:numel (varargin)
    P.(varargin{i}) = varargin{i+1};
  end

  % --- derived quantities (recomputed after overrides) -------------------
  P.centers = linspace (0, P.d_meas, P.N);
  % 2*sigma = zone spacing makes the triangular partition sum to unity
  P.sigma   = (P.d_meas / (P.N - 1)) / 2;
  P.w1_c     = linspace (P.w1_min, P.w1_max, P.N);   % increasing in d
  P.w2_c     = linspace (P.w2_max, P.w2_min, P.N);   % decreasing in d
  P.w1c      = P.w1_c;  % Alias for backwards compatibility
  P.w2c      = P.w2_c;  % Alias for backwards compatibility
  
  % Bounds for Proposal 2 adaptation
  P.c_min    = 0.0;      % minimum center value
  P.c_max    = P.d_meas; % maximum center value

  % gradient bound G_r over the safe set  [r_safety, d0]   (Eq. Gr-def)
  dd     = linspace (P.r_safety, P.d0, 20001);
  P.G_r  = max (fapf_gradUr_mag (dd, P));

  % residual radius (Eq. residual-radius)
  P.R    = P.w2_max * P.k_r * P.G_r / (P.w1_min * P.k_a);

  % class-K gain.  Eq. (alpha0-choice) only certifies the two endpoints of
  % [r_safety, d0]; the interior can still violate the barrier condition, so
  % alpha0 is taken as the true supremum over the interval.
  P.alpha0_formula = P.w1_max * P.k_a * P.D_max / (P.d0 - P.r_safety);
  ds  = linspace (P.r_safety, P.d0, 4001);
  num = zeros (size (ds));
  for i = 1:numel (ds)
    [a1, a2] = fapf_weights (ds(i), P);
    num(i) = a1 * P.k_a * P.D_max - a2 * P.k_r * fapf_gradUr_mag (ds(i), P);
  end
  m = ds > P.r_safety + 1e-9;
  P.alpha0_required = max (num(m) ./ (ds(m) - P.r_safety));
  P.alpha0 = max (P.alpha0_formula, P.alpha0_required);
end
