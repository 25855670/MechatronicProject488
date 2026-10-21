function exportSignals(stem, name, L, wheel, P, r, b)
%EXPORTSIGNALS  Write one vehicle's run to CSV for demo_dashboard.
%
%   <stem>_<name>_host.csv   one row per control tick: pose, commands and
%                            controller internals
%   <stem>_<name>_wheel.csv  one row per telemetry row: wheel setpoints as
%                            received, measured speeds and PI outputs [V]
%   <stem>_<name>_path.csv   the commanded waypoints [m]
%
%   L is the run log, wheel the output of alignTelemetry, P the waypoints,
%   r the wheel radius and b the track width [m].

% Speed and turn rate from the camera, only between consecutive seen frames
n = numel(L.t);
vMeas = nan(n, 1);  wMeas = nan(n, 1);
for i = 2:n
    dt = L.t(i) - L.t(i-1);
    if L.seen(i) && L.seen(i-1) && dt > 1e-6
        vMeas(i) = norm([L.x(i)-L.x(i-1), L.y(i)-L.y(i-1)]) / dt;
        wMeas(i) = ArucoTracker.wrapPi(L.th(i) - L.th(i-1)) / dt;
    end
end

T = table(L.t, double(L.seen), L.x, L.y, L.th, ...
          L.v, L.w, L.wL, L.wR, vMeas, wMeas, ...
          L.kappa, L.xte, L.alpha, L.tgt(:,1), L.tgt(:,2), ...
    'VariableNames', {'t_s','seen','x_m','y_m','theta_rad', ...
          'v_cmd_mps','omega_cmd_radps','wL_ref_radps','wR_ref_radps', ...
          'v_meas_mps','omega_meas_radps', ...
          'kappa_cmd_1pm','xte_m','alpha_rad','target_x_m','target_y_m'});
writetable(T, stem + "_" + name + "_host.csv");

W = table(wheel.t, wheel.tMs, wheel.spL, wheel.wL, wheel.uL, ...
          wheel.spR, wheel.wR, wheel.uR, wheel.flags, ...
          (wheel.wL + wheel.wR)*r/2, (wheel.wR - wheel.wL)*r/b, ...
    'VariableNames', {'t_s','t_vehicle_ms','wL_ref_rx_radps', ...
          'wL_act_radps','uL_V','wR_ref_rx_radps','wR_act_radps', ...
          'uR_V','flags','v_enc_mps','omega_enc_radps'});
writetable(W, stem + "_" + name + "_wheel.csv");

writetable(table(P(:,1), P(:,2), 'VariableNames', {'x_m','y_m'}), ...
           stem + "_" + name + "_path.csv");
end
