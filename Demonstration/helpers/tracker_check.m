%% Tracker check - before a run, shows live what the overhead camera and ArUco tracker
%% see: the locked arena outline and axes, and each vehicle's marker, heading and axle
%% centre. Also measures the pose noise with the vehicles still. Q or close the window to finish.
helpersDir = fileparts(mfilename('fullpath'));
addpath(helpersDir);

%% ================= CAMERA SETTINGS =================
% Camera name, or a substring of it
CAM_NAME = "USB";
% Capture resolution - must match the calibration [px]
RESOLUTION = "1920x1080";
% Lens calibration from camera_calibrate.m, in the Demonstration folder
CALIB = fullfile(helpersDir, '..', 'camera_params.mat');

%% ================= ARENA SETTINGS =================
% Arena size [W H], measured between the corner markers' inner corners [mm]
AREA_MM = [2400 1700];
% Corner marker ids at (0,0), (W,0), (W,H), (0,H)
CORNER_IDS = [1 4 3 2];
% ArUco family, "" = auto-detect at lock
FAMILY = "";

%% ================= VEHICLE MARKER SETTINGS =================
% Vehicle marker ids
VEH_MARKERS = [5 6 7];
% Height of the marker face above the floor [mm]
MARKER_H_MM = 49;
% Marker centre relative to the axle centre [x forward, y left] [mm] (marker behind the axle)
MARKER_OFF_MM = [-110 0];

%% ================= CHECK SETTINGS =================
% Frames used to measure the pose noise
NOISE_FRAMES = 60;

%% ================= OPEN AND LOCK =================
t = ArucoTracker();
t.CamName      = CAM_NAME;
t.Resolution   = RESOLUTION;
t.Calib        = CALIB;
t.AreaSize     = AREA_MM / 1000;
t.CornerIds    = CORNER_IDS;
t.Family       = FAMILY;
t.MarkerHeight = MARKER_H_MM / 1000;
t.MarkerOffset = MARKER_OFF_MM / 1000;
t.open();
t.lock();

%% ================= NOISE FLOOR =================
fprintf('\nmeasuring pose noise over %d frames - keep the vehicles still\n', NOISE_FRAMES);
nM  = numel(VEH_MARKERS);
% Marker [x y yaw] per frame and per vehicle [m, rad]
buf = nan(NOISE_FRAMES, 3, nM);
for i = 1:NOISE_FRAMES
    s = t.read(VEH_MARKERS);
    for k = find([s.seen])
        buf(i, :, k) = [s(k).xyMarker, s(k).thetaMarker];
    end
end
for k = 1:nM
    b = buf(~isnan(buf(:, 1, k)), :, k);
    if size(b, 1) < 5
        fprintf('  id %d  seen in only %d of %d frames\n', VEH_MARKERS(k), size(b, 1), NOISE_FRAMES);
    else
        fprintf('  id %d  sd x %.2f mm   y %.2f mm   yaw %.2f deg\n', VEH_MARKERS(k), ...
                1000*std(b(:, 1)), 1000*std(b(:, 2)), rad2deg(std(unwrap(b(:, 3)))));
    end
end

%% ================= LIVE OVERLAY =================
fprintf('\nlive view - Q or close the window to finish\n');
fig = figure('Name', 'tracker check', 'Color', 'w', 'NumberTitle', 'off');
setappdata(fig, 'key', '');
fig.KeyPressFcn = @(src, evt) setappdata(src, 'key', evt.Key);

% Arena outline and 200 mm world axes at the origin, fixed once the frame is locked
pArea = worldToPix(t, [t.areaCorners(), zeros(5, 1)]);
pAxes = worldToPix(t, [0 0 0; 0.2 0 0; 0 0.2 0]);

hIm = imshow(t.snap()); hold on;
plot(pArea(:, 1), pArea(:, 2), 'c-', 'LineWidth', 2);
plot(pAxes([1 2], 1), pAxes([1 2], 2), 'r-', 'LineWidth', 3);
plot(pAxes([1 3], 1), pAxes([1 3], 2), 'g-', 'LineWidth', 3);
text(pAxes(2, 1), pAxes(2, 2), ' +x', 'Color', 'r', 'FontWeight', 'bold');
text(pAxes(3, 1), pAxes(3, 2), ' +y', 'Color', 'g', 'FontWeight', 'bold');
text(pAxes(1, 1), pAxes(1, 2), sprintf(' 0,0 (id %d)', CORNER_IDS(1)), ...
     'Color', 'c', 'FontWeight', 'bold');

% Per vehicle: marker outline, 150 mm heading line, marker centre (+) and axle centre (o),
% created once and updated in place each frame
hQuad = gobjects(1, nM); hHead = gobjects(1, nM); hMC = gobjects(1, nM); hAC = gobjects(1, nM);
for k = 1:nM
    hQuad(k) = plot(nan, nan, 'y-', 'LineWidth', 1.5);
    hHead(k) = plot(nan, nan, 'm-', 'LineWidth', 2.5);
    hMC(k)   = plot(nan, nan, 'y+', 'MarkerSize', 12, 'LineWidth', 2);
    hAC(k)   = plot(nan, nan, 'mo', 'MarkerSize', 12, 'LineWidth', 2);
end
hold off;
hTitle = title('', 'Interpreter', 'none', 'FontSize', 9);

nFrame = 0; nSeen = 0; tL = tic;
while true
    I = t.snap();
    s = t.read(VEH_MARKERS, I);
    nFrame = nFrame + 1;
    set(hIm, 'CData', I);

    txt = cell(1, nM);
    for k = 1:nM
        if ~s(k).seen
            set([hQuad(k) hHead(k) hMC(k) hAC(k)], 'XData', nan, 'YData', nan);
            txt{k} = sprintf('id %d  NOT SEEN', s(k).id);
            continue;
        end
        nSeen = nSeen + 1;
        q  = s(k).pixels;
        hp = worldToPix(t, [s(k).xy 0; s(k).xy + 0.15*[cos(s(k).theta) sin(s(k).theta)] 0]);
        mc = worldToPix(t, [s(k).xyMarker 0]);
        set(hQuad(k), 'XData', q([1:4 1], 1), 'YData', q([1:4 1], 2));
        set(hHead(k), 'XData', hp(:, 1),      'YData', hp(:, 2));
        set(hMC(k),   'XData', mc(1),         'YData', mc(2));
        set(hAC(k),   'XData', hp(1, 1),      'YData', hp(1, 2));
        txt{k} = sprintf('id %d  axle [%.0f %.0f] mm  heading %.1f deg', ...
                         s(k).id, 1000*s(k).xy, rad2deg(s(k).theta));
    end

    set(hTitle, 'String', [{sprintf('%s  |  %.1f fps  |  markers seen %.0f%%', t.Family, ...
                                    nFrame/toc(tL), 100*nSeen/(nFrame*nM))}, txt]);
    drawnow limitrate;

    if ~isvalid(fig) || strcmp(getappdata(fig, 'key'), 'q'), break; end
end
if isvalid(fig), close(fig); end
t.close();

%% ================= LOCAL FUNCTIONS =================
function p = worldToPix(t, P3)
% World points [m] -> raw camera pixels (lens distortion applied), for the overlay.
    p = world2img(P3, t.Extr, t.Intrinsics, ApplyDistortion=true);
end
