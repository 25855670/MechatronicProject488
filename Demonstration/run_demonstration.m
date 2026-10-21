%% Pure pursuit demonstration - drives up to three vehicles along the planner's
%% routes, using the overhead ArUco camera for pose and Bluetooth for wheel
%% speed commands, then logs the run and draws the dashboard.
%% Run Simulation/route_planner.m first to write the waypoint CSV.
clear; clc; close all;
cd(fileparts(mfilename('fullpath')));
addpath('helpers');

%% ================= ROUTES =================
% Waypoint file written by route_planner.m
CSV_FILE = fullfile('..', 'Simulation', 'vehicle_paths.csv');

% Planner area [xmin ymin xmax ymax] [m] - same as route_planner.m
PLAN_AREA = [0 0 2.4 1.6];

% Planner grid cell size [m] - same as route_planner.m
CELL_SIZE = 0.4;

%% ================= FLEET =================
% Vehicles to drive (vehicle k follows the planner's v<k> route)
VEH_ENABLE = [true true true];

% Paired HC-06 Bluetooth names (or COM ports)
VEH_NAMES = ["VEH1" "VEH2" "VEH3"];

% ArUco marker id on each vehicle
VEH_MARKERS = [5 6 7];

% Axle centre -> marker centre in the vehicle frame, +x forward [mm]
MARKER_OFF_MM = [-110 0];

% Floor to the printed face of the vehicle markers [mm]
MARKER_H_MM = 49;

%% ================= ARENA AND CAMERA =================
% Arena size [W H], inner corner to inner corner of the corner markers [mm]
AREA_MM = [2400 1700];

% Corner marker ids at (0,0), (W,0), (W,H), (0,H)
CORNER_IDS = [1 4 3 2];

% Camera calibration from helpers/camera_calibrate.m
CALIB = "camera_params.mat";

% Camera resolution - must match the calibration
RESOLUTION = "1920x1080";

% Substring of the camera name
CAM_NAME = "USB";

%% ================= CHASSIS =================
% Wheel radius [m]
WHEEL_R = 0.0325;
% Track width, centre-to-centre of the wheels [m]
TRACK_B = 0.136;
% Highest wheel speed commanded [rad/s]
W_MAX = 15.0;
% Lowest wheel speed the motors hold reliably [rad/s]
W_WHEEL_MIN = 5.0;
% Wheel speed PI gains [kp ki], pushed to every vehicle at connect
GAINS = [0.18 2.5];

%% ================= PURE PURSUIT =================
% Lookahead distance [m]
LOOKAHEAD = 0.4;
% Forward speed of the point midway between the wheels [m/s]
V_DESIRED = 0.25;
% Goal capture radius [m]
GOAL_TOL = 0.10;
% Ticks spent moving away from a missed goal before stopping at the closest point
GOAL_RECEDE = 5;

%% ================= LOOP AND SAFETY =================
% Control loop rate ceiling - the camera usually sets the real rate [Hz]
CTRL_HZ = 15;
% Vehicle telemetry period [ms]
TELEM_MS = 50;
% Run time limit [s]
T_MAX = 45;
% Missed camera frames a vehicle coasts through
MISS_HOLD = 3;
% Consecutive missed frames that stop a vehicle
MISS_LIMIT = 15;
% Furthest a vehicle may start from its first waypoint [m]
START_TOL = 0.30;
% Countdown before driving [s]
COUNTDOWN = 3;
% How far outside the arena a vehicle may go before it is stopped [mm]
FENCE_MM = 500;
% Radius round each path's start and end where the fence is not applied [mm]
FENCE_GRACE_MM = 50;
% Centre-to-centre distance at which two vehicles are both stopped [mm]
PROX_STOP_MM = 400;

%% ================= RECORDING =================
% Record the camera feed to <run>.mp4 (lens distortion removed after the run)
RECORD_VIDEO = true;
% MPEG-4 quality, 0..100
VIDEO_QUALITY = 100;

%% ================= DERIVED SETTINGS =================
sel   = find(VEH_ENABLE);
nV    = numel(sel);
areaM = AREA_MM / 1000;

% Turn rate limit at V_DESIRED from the wheel speed limits, given to pure
% pursuit as a curvature limit so every command keeps both wheels in range
wMaxTurn = maxTurnRate(V_DESIRED, TRACK_B, WHEEL_R, W_WHEEL_MIN, W_MAX);
if wMaxTurn <= 0
    error('V_DESIRED = %.3f m/s leaves no room to turn within the wheel limits %.1f-%.1f rad/s.', ...
          V_DESIRED, W_WHEEL_MIN, W_MAX);
end
rMin      = V_DESIRED / wMaxTurn;
KAPPA_MAX = 1 / rMin;

% Arrival by drawing level with the goal only counts within this radius [m]
goalCapture = max(4*GOAL_TOL, 2*rMin);

fenceM = FENCE_MM / 1000;
graceM = FENCE_GRACE_MM / 1000;
proxM  = PROX_STOP_MM / 1000;

fprintf('turn limit %.2f rad/s at %.2f m/s -> minimum radius %.0f mm\n\n', ...
        wMaxTurn, V_DESIRED, 1000*rMin);

%% ================= PATHS =================
VS = struct('name', cellstr(VEH_NAMES(sel)), 'marker', num2cell(VEH_MARKERS(sel)));
fprintf('waypoints : %s\n', CSV_FILE);
for k = 1:nV
    [VS(k).P, margin] = mapPlanPath(CSV_FILE, sel(k), PLAN_AREA, areaM);
    P = VS(k).P;
    fprintf('%-5s %2d waypoints, start [%.0f %.0f] mm, goal [%.0f %.0f] mm\n', ...
            VS(k).name, size(P,1), 1000*P(1,:), 1000*P(end,:));
end
fprintf('\n');

%% ================= CAMERA =================
tr = ArucoTracker();
tr.Calib        = CALIB;
tr.Resolution   = RESOLUTION;
tr.CamName      = CAM_NAME;
tr.AreaSize     = areaM;
tr.CornerIds    = CORNER_IDS;
tr.MarkerHeight = MARKER_H_MM / 1000;
tr.MarkerOffset = MARKER_OFF_MM / 1000;

trGuard = onCleanup(@() tr.close());
tr.open();
tr.lock();
fprintf('\n');

%% ================= START POSITIONS =================
% Every vehicle must be visible, near its first waypoint and inside the fence
badStart = '';
for k = 1:nV
    s = tr.read(VS(k).marker);
    for t = 1:11
        if s.seen, break; end
        s = tr.read(VS(k).marker);
    end
    if ~s.seen
        badStart = [badStart sprintf('  %s (marker %d) is not visible\n', ...
                    VS(k).name, VS(k).marker)]; %#ok<AGROW>
        continue
    end

    d0 = norm(s.xy - VS(k).P(1, :));
    fprintf('%-5s at [%.0f %.0f] mm, %.0f mm from its start\n', ...
            VS(k).name, 1000*s.xy, 1000*d0);
    if d0 > START_TOL
        badStart = [badStart sprintf('  %s is %.0f mm out - move it to [%.0f %.0f] mm\n', ...
                    VS(k).name, 1000*d0, 1000*VS(k).P(1,:))]; %#ok<AGROW>
    elseif outsideFence(s.xy, VS(k).P, areaM, fenceM, graceM)
        badStart = [badStart sprintf('  %s is outside the fence\n', VS(k).name)]; %#ok<AGROW>
    end
end
if ~isempty(badStart)
    error('Not ready to start (start tolerance %.0f mm):\n%s', 1000*START_TOL, badStart);
end
fprintf('\n');

%% ================= CONNECT =================
f = FleetLink(VEH_NAMES(sel));
f.Ids   = sel;
f.Gains = repmat(GAINS, nV, 1);
fGuard  = onCleanup(@() f.close());

fprintf('connecting to %s...\n', strjoin(VEH_NAMES(sel), ', '));
f.connect();
if ~all(f.Open)
    error('%s did not open. Check each HC-06 is paired, powered and not in use.', ...
          strjoin(VEH_NAMES(sel(~f.Open)), ', '));
end
if ~all(f.Confirmed)
    error('%s did not confirm its id - power-cycle the module and re-run.', ...
          strjoin(VEH_NAMES(sel(~f.Confirmed)), ', '));
end

f.setTelemetry(TELEM_MS);

%% ================= RUN FILES AND VIDEO =================
if ~exist('runs', 'dir'), mkdir('runs'); end
stem = fullfile("runs", "run_" + string(datetime('now', 'Format', 'yyyyMMdd_HHmmss')));

vidT = [];
if RECORD_VIDEO
    vw = VideoWriter(char(stem + ".mp4"), 'MPEG-4');
    vw.Quality   = VIDEO_QUALITY;
    vw.FrameRate = CTRL_HZ;
    open(vw);
    vidGuard = onCleanup(@() close(vw));
    vidPath  = fullfile(vw.Path, vw.Filename);

    % The first frame loads the encoder (~0.4 s), so write it before the loop
    writeVideo(vw, tr.snap());
    vidT(end+1) = 0;
end

%% ================= COUNTDOWN =================
% Heartbeats keep the vehicles' link watchdogs from timing out meanwhile
fprintf('CLEAR THE AREA. Driving %s in ', strjoin(VEH_NAMES(sel), ', '));
for s = COUNTDOWN:-1:1
    fprintf('%d ', s);
    for j = 1:4
        f.heartbeat();
        f.telemetry();
        pause(0.25);
    end
end
fprintf('go\n\n');

%% ================= CONTROL LOOP =================
n = ceil(T_MAX * CTRL_HZ) + 64;
for k = 1:nV
    pp = controllerPurePursuitFromStart;
    pp.Waypoints             = VS(k).P;
    pp.LookaheadDistance     = LOOKAHEAD;
    pp.DesiredLinearVelocity = V_DESIRED;
    pp.MaxCurvature          = KAPPA_MAX;
    VS(k).pp = pp;

    VS(k).log = struct('t', nan(n,1), 'seen', false(n,1), 'x', nan(n,1), ...
             'y', nan(n,1), 'th', nan(n,1), 'v', nan(n,1), 'w', nan(n,1), ...
             'wL', nan(n,1), 'wR', nan(n,1), 'kappa', nan(n,1), ...
             'xte', nan(n,1), 'alpha', nan(n,1), 'tgt', nan(n,2));
    VS(k).i      = 0;
    VS(k).done   = false;
    VS(k).why    = 'timeout';
    VS(k).miss   = 0;
    VS(k).lastW  = [0 0];
    VS(k).lastOm = 0;
    VS(k).dBest  = Inf;     % closest approach to the goal so far
    VS(k).nAway  = 0;       % ticks since the closest approach
    VS(k).armed  = false;   % has been within goalCapture on the final approach
    VS(k).remMin = Inf;     % least path left to drive so far
end
markers = [VS.marker];

dt    = 1 / CTRL_HZ;
tRun  = tic;
f.Clock = tRun;         % telemetry timestamps on the loop clock
f.Log   = [];           % keep only telemetry from the run itself
tNext = 0;

while toc(tRun) < T_MAX && ~all([VS.done])
    tNow = toc(tRun);

    % One camera frame serves the detector, the recording and every vehicle
    I    = tr.snap();
    sAll = tr.read(markers, I);
    if RECORD_VIDEO
        writeVideo(vw, I);
        vidT(end+1) = tNow; %#ok<SAGROW>
    end

    % Proximity stop: two running vehicles closer than PROX_STOP_MM both stop
    for a = 1:nV-1
        if VS(a).done || ~sAll(a).seen, continue; end
        for b = a+1:nV
            if VS(b).done || ~sAll(b).seen, continue; end
            dAB = norm(sAll(a).xy - sAll(b).xy);
            if dAB < proxM
                for q = [a b]
                    VS(q).why  = 'proximity';
                    VS(q).done = true;
                end
                fprintf('  [%5.2f s] PROXIMITY STOP: %s and %s at %.0f mm\n', ...
                        tNow, VS(a).name, VS(b).name, 1000*dAB);
            end
        end
    end

    Wcmd = zeros(nV, 2);
    for k = 1:nV
        if VS(k).done, continue; end
        s = sAll(k);

        if s.seen
            VS(k).miss = 0;
            pose = [s.xy s.theta];
            [v, w, kappa, alpha, lookPt] = ppCommand(VS(k).pp, pose);
            W = FleetLink.wheelSpeeds(v, w, WHEEL_R, TRACK_B);
            VS(k).lastOm = w;

            % Arrival is only judged on the final approach, once less path
            % is left than goalCapture
            VS(k).remMin = min(VS(k).remMin, pathRemaining(s.xy, VS(k).P));
            onFinal = VS(k).remMin < goalCapture;
            [atGoal, gWhy] = goalReached(s.xy, VS(k).P, GOAL_TOL, goalCapture, onFinal);
            dGoal = norm(s.xy - VS(k).P(end, :));

            if atGoal
                VS(k).why = gWhy;  VS(k).done = true;
            else
                % A forward-only vehicle that misses the goal would circle
                % it, so stop once it has moved away for GOAL_RECEDE ticks
                if dGoal < goalCapture && onFinal
                    VS(k).armed = true;
                    if dGoal < VS(k).dBest - 1e-4
                        VS(k).dBest = dGoal;  VS(k).nAway = 0;
                    else
                        VS(k).nAway = VS(k).nAway + 1;
                    end
                elseif VS(k).armed
                    VS(k).nAway = VS(k).nAway + 1;
                end

                if VS(k).armed && VS(k).nAway >= GOAL_RECEDE
                    VS(k).why = 'closest';  VS(k).done = true;
                elseif outsideFence(s.xy, VS(k).P, areaM, fenceM, graceM)
                    VS(k).why = 'fence';    VS(k).done = true;
                end
            end
            if VS(k).done, W = [0 0]; end

            i = VS(k).i + 1;  VS(k).i = i;
            L = VS(k).log;
            L.t(i) = tNow;  L.seen(i) = true;
            L.x(i) = s.xy(1);  L.y(i) = s.xy(2);  L.th(i) = s.theta;
            L.v(i) = v;  L.w(i) = w;  L.wL(i) = W(1);  L.wR(i) = W(2);
            L.kappa(i) = kappa;  L.alpha(i) = alpha;  L.tgt(i, :) = lookPt;
            L.xte(i) = pointToPolyline(s.xy, VS(k).P);
            VS(k).log = L;
        else
            % Marker not seen: coast for MISS_HOLD frames with the turn
            % decaying toward straight, then hold still, then give up
            VS(k).miss = VS(k).miss + 1;
            if VS(k).miss > MISS_LIMIT
                W = [0 0];
                VS(k).why  = 'lost';
                VS(k).done = true;
            elseif VS(k).miss <= MISS_HOLD
                fade  = 1 - VS(k).miss / (MISS_HOLD + 1);
                vHold = mean(VS(k).lastW) * WHEEL_R;
                W = FleetLink.wheelSpeeds(vHold, VS(k).lastOm * fade, WHEEL_R, TRACK_B);
            else
                W = [0 0];
                VS(k).lastOm = 0;
            end

            i = VS(k).i + 1;  VS(k).i = i;
            VS(k).log.t(i) = tNow;
            VS(k).log.wL(i) = W(1);  VS(k).log.wR(i) = W(2);
        end

        VS(k).lastW = W;
        Wcmd(k, :) = W;
    end

    f.stepAll(Wcmd);
    f.telemetry();

    % Pace against the run clock rather than sleeping a fixed dt
    tNext = tNext + dt;
    slack = tNext - toc(tRun);
    if slack > 0, pause(slack); else, tNext = toc(tRun); end
end

f.stopAll();
pause(0.3);
f.telemetry();
tel = f.Log;
f.setTelemetry(100);
f.close();

% Close the recording, then remove the lens distortion from it
clear vidGuard
if RECORD_VIDEO
    fprintf('undistorted %d video frames\n', undistortVideoFile(vidPath, tr.Intrinsics));
end

for k = 1:nV
    fn = fieldnames(VS(k).log);
    for q = 1:numel(fn)
        VS(k).log.(fn{q}) = VS(k).log.(fn{q})(1:VS(k).i, :);
    end
end

%% ================= RESULTS =================
fprintf('\n--- run finished ---\n');
for k = 1:nV
    L = VS(k).log;
    j = find(L.seen, 1, 'last');
    tEnd = 0;  dEnd = NaN;
    if ~isempty(j)
        tEnd = L.t(j);
        dEnd = norm([L.x(j) L.y(j)] - VS(k).P(end, :));
    end
    fprintf('%-5s %-9s after %4.1f s, %3.0f mm from the goal, cross-track mean %.0f / max %.0f mm\n', ...
            VS(k).name, upper(VS(k).why), tEnd, 1000*dEnd, ...
            1000*mean(L.xte, 'omitnan'), 1000*max(L.xte));
end

runData = struct('when', datetime('now'), 'csv', CSV_FILE, 'areaM', areaM, ...
                 'telemAll', tel, 'videoT', vidT(:));
for k = 1:nV
    d.name  = VS(k).name;
    d.why   = VS(k).why;
    d.path  = VS(k).P;
    d.log   = VS(k).log;
    d.telem = [];
    if ~isempty(tel), d.telem = tel(tel(:,1) == sel(k), :); end
    d.wheel = alignTelemetry(d.telem);
    runData.veh(k) = d;

    exportSignals(stem, d.name, d.log, d.wheel, d.path, WHEEL_R, TRACK_B);
end
save(stem + ".mat", 'runData');
fprintf('saved %s.mat and its CSV files\n', stem);

%% ================= DASHBOARD =================
dashOpts = struct("area", [PLAN_AREA(1:2); PLAN_AREA(3:4)], "cellSize", CELL_SIZE, ...
                  "safeDist", proxM, "planOffset", PLAN_AREA(1:2) - margin);
demo_dashboard(stem, dashOpts);
