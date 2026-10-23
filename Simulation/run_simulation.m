%% Pure pursuit simulation - sets the model parameters, runs
%% PurePursuitControlledRobot on the planner's waypoints and draws the dashboard.
%% Run route_planner.m first to write the waypoint CSV.
clear; clc; close all;
cd(fileparts(mfilename('fullpath')));

%% ================= SIMULATION SETTINGS =================
% Waypoint file written by route_planner.m
CSV_FILE = 'vehicle_paths.csv';

% Simulation stop time [s] (the run ends earlier once all vehicles reach their goal)
STOP_TIME = 30;

% Movement area corners [xmin ymin; xmax ymax] [m] - same as route_planner.m
AREA = [0 0; 2.4 1.6];

% Planner grid cell size [m] - same as route_planner.m
CELL_SIZE = 0.4;

% Centre-to-centre separation drawn on the dashboard [m]
SAFE_DIST = 0.40;

%% ================= MODEL PARAMETERS =================
% Controller sample time [s]
Ts = 0.02;
% Plant integration step [s]
Ts_fast = 2e-5;

% Camera frame period [s]
Ts_cam = 1/25;
% Camera pipeline latency [s]
CAM_LAT = 0.08;

% Camera position noise, 1-sigma [m]
sig_xy = 0.002;
% Camera heading noise, 1-sigma [rad]
sig_th = 0.035;

% Camera calibration bias in x [m], y [m] and heading [rad]
bias_x  = 0.000;
bias_y  = 0.000;
bias_th = 0.000;

% Gearbox ratio
N = 48;
% Armature resistance [ohm]
Ra = 7.953487304;
% Motor constant [V*s/rad]
K = 0.002811691;
% Motor driver saturation voltage [V]
E_max = 6.6;
% Left motor driver voltage map, slope and offset
m_L = 1.1763;
c_L = 1.5813;
% Right motor driver voltage map, slope and offset
m_R = 1.1635;
c_R = 1.8775;

% Effective vehicle mass [kg]
mass = 0.85;
% Viscous friction [N*s/m]
b = 4.50;

% Wheel radius [m]
R = 0.0325;
R_wheel = 0.065/2;
% Track width, centre-to-centre of the wheels [m]
L_track = 0.136;

% Wheel speed PI gains
Kp = 0.18;
Ki = 2.5;

% Pure pursuit forward speed [m/s]
v_desired = 0.25;
% Pure pursuit maximum curvature [1/m]
max_curve = 1/0.198;
% Pure pursuit lookahead distance [m]
lookahead = 0.4;
% Goal capture radius [m]
tol = 0.1;

%% ================= WAYPOINTS AND INITIAL POSES =================
csv = readmatrix(CSV_FILE);
W = cell(1,3);
for k = 1:3
    kk   = ~isnan(csv(:, 2*k-1));
    W{k} = [csv(kk, 2*k-1), csv(kk, 2*k)];
end
fprintf('waypoints : %s\n', CSV_FILE);

waypoints_v1 = W{1};  goal_v1 = waypoints_v1(end,:).';
waypoints_v2 = W{2};  goal_v2 = waypoints_v2(end,:).';
waypoints_v3 = W{3};  goal_v3 = waypoints_v3(end,:).';

% Each vehicle starts on its first waypoint, facing along its first segment
pose0_v1 = [W{1}(1,:).'; atan2(W{1}(2,2)-W{1}(1,2), W{1}(2,1)-W{1}(1,1))];
pose0_v2 = [W{2}(1,:).'; atan2(W{2}(2,2)-W{2}(1,2), W{2}(2,1)-W{2}(1,1))];
pose0_v3 = [W{3}(1,:).'; atan2(W{3}(2,2)-W{3}(1,2), W{3}(2,1)-W{3}(1,1))];

%% ================= RUN THE SIMULATION =================
out = sim("PurePursuitControlledRobot", "StopTime", num2str(STOP_TIME));

%% ================= DASHBOARD =================
dashOpts = struct("area", AREA, "cellSize", CELL_SIZE, "safeDist", SAFE_DIST, ...
                  "R_wheel", R_wheel, "L_track", L_track, "lookahead", lookahead);
sim_dashboard(out, {waypoints_v1, waypoints_v2, waypoints_v3}, dashOpts);
