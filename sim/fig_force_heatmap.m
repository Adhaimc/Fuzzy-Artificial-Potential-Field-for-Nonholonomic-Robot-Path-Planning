function fig_force_heatmap ()
% FIG_FORCE_HEATMAP: Spatial force-magnitude heatmaps for Fuzzy-APF
% (implemented design, P = fapf_params()) vs. classical Khatib APF
% (unbounded inverse-square repulsion), over the 100-obstacle field
% described in Sec. sec:experiments (fig:force-analysis-heatmap).

  P = fapf_params ();
  goal = [0, 0];
  nobs = 100;
  rw = 9;   % workspace radius (paper: "9 m radius circular workspace")

  rand ('state', 2026);
  obs = zeros (nobs, 3);
  k = 0;
  while (k < nobs)
    ang = 2*pi*rand ();
    rad = rw * sqrt (rand ());
    c = [rad*cos(ang), rad*sin(ang)];
    r = (0.2 + 0.4*rand ()) / 2;
    if (norm (c - goal) <= 1.5), continue; end
    k++;
    obs(k,:) = [c, r];
  end

  % Classical (Khatib) fixed-gain, unbounded inverse-square baseline
  % (Table tab:repulsive-models, "Inverse-square (Khatib)" row).
  k_a_k = 1.0; k_r_k = 10.0; d0_k = 2.0;

  ng = 140;
  gv = linspace (-rw-1, rw+1, ng);
  [X, Y] = meshgrid (gv, gv);
  Ffuzzy  = nan (ng, ng);
  Fkhatib = nan (ng, ng);

  for i = 1:ng
    for j = 1:ng
      q = [X(i,j), Y(i,j)];
      if (any (sqrt (sum ((obs(:,1:2) - q).^2, 2)) < obs(:,3)))
        continue;   % inside an obstacle body
      end

      [d, nhat] = fapf_dobs (q, obs);

      % Fuzzy-APF is only guaranteed bounded on the safe set d_obs>=r_safety
      % (thm:collision-safety); points closer than that are never visited by
      % the closed loop and are masked here rather than shown as an
      % artificial 1/d^2-type spike.
      if (d >= P.r_safety)
        [Ff, ~] = fapf_force (q, goal, obs, P);
        Ffuzzy(i,j) = norm (Ff);
      end

      Fatt = -k_a_k * (q - goal);
      if (d < d0_k && d > 1e-9)
        Frep = k_r_k * (1/d^2 - 1/d0_k^2) * nhat;
      else
        Frep = [0, 0];
      end
      Fkhatib(i,j) = norm (Fatt + Frep);
    end
  end

  peak_fuzzy  = max (Ffuzzy(~isnan (Ffuzzy(:))));
  peak_khatib = max (Fkhatib(:));
  printf ('Fuzzy-APF (implemented design k_a=%.1f,k_r=%.1f,d0=%.1f,r_safety=%.1f): peak |F| = %.2f N\n', ...
          P.k_a, P.k_r, P.d0, P.r_safety, peak_fuzzy);
  printf ('Khatib (classical, unbounded k_a=%.1f,k_r=%.1f):  peak |F| = %.4g N\n', ...
          k_a_k, k_r_k, peak_khatib);
  printf ('Dynamic range (Khatib peak / Fuzzy peak): %.4g\n', peak_khatib / peak_fuzzy);

  fig = figure ('Position', [100, 100, 1000, 480], 'Color', 'white');
  th = linspace (0, 2*pi, 32);

  subplot (1, 2, 1);
  surf (X, Y, Ffuzzy, 'EdgeColor', 'none'); view (0, 90); colormap (jet);
  hold on;
  for k = 1:nobs
    plot (obs(k,1)+obs(k,3)*cos(th), obs(k,2)+obs(k,3)*sin(th), 'k-', 'LineWidth', 1);
  end
  plot (goal(1), goal(2), 'r*', 'MarkerSize', 14, 'LineWidth', 2);
  axis equal; axis ([-rw-1, rw+1, -rw-1, rw+1]);
  caxis ([0, peak_fuzzy]);
  c1 = colorbar; ylabel (c1, 'Force (N)');
  title (sprintf ('Fuzzy-APF: peak %.1f N', peak_fuzzy));
  xlabel ('X (m)'); ylabel ('Y (m)');

  subplot (1, 2, 2);
  surf (X, Y, Fkhatib, 'EdgeColor', 'none'); view (0, 90); colormap (jet);
  hold on;
  for k = 1:nobs
    plot (obs(k,1)+obs(k,3)*cos(th), obs(k,2)+obs(k,3)*sin(th), 'k-', 'LineWidth', 1);
  end
  plot (goal(1), goal(2), 'r*', 'MarkerSize', 14, 'LineWidth', 2);
  axis equal; axis ([-rw-1, rw+1, -rw-1, rw+1]);
  caxis ([0, peak_khatib]);
  c2 = colorbar; ylabel (c2, 'Force (N)');
  title (sprintf ('Khatib (classical): peak %.3g N', peak_khatib));
  xlabel ('X (m)'); ylabel ('Y (m)');

  print (fig, 'force_analysis_4panel.png', '-dpng', '-r150');
  printf ('Saved: sim/force_analysis_4panel.png\n');
end
