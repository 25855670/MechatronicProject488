classdef ArucoTracker < handle
%ARUCOTRACKER  Overhead-camera pose of ArUco-tagged vehicles, in metres.
%
%   Four corner markers on the floor define the world frame: (0,0) is the
%   corner of CornerIds(1) nearest the middle of the arena and AreaSize is
%   measured between those inner corners. Vehicle markers sit MarkerHeight
%   above the floor, so the floor plane is lifted to that height before
%   marker corners are mapped to world coordinates.
%
%   t = ArucoTracker();
%   t.Calib = "camera_params.mat";  t.AreaSize = [2.4 1.7];
%   t.open(); t.lock();
%   s = t.read([5 6 7]);            % -> .seen .xy .theta per marker

    properties
        % Webcam name, a substring of one, or an index
        CamName    = ""
        % Capture resolution, e.g. "1920x1080" (must match the calibration)
        Resolution = ""
        % .mat file holding cameraParameters/cameraIntrinsics (camera_calibrate.m)
        Calib = ""
        % Arena size [W H] between the inner marker corners [m]
        AreaSize  = [2.400 1.700]
        % Corner marker ids at (0,0), (W,0), (W,H), (0,H)
        CornerIds = [1 2 3 4]
        % Allowed corner-fit residual before the frame is flagged [mm]
        FrameTolMM = 100
        % Floor to the printed face of the vehicle markers [m]
        MarkerHeight = 0.0
        % Axle centre -> marker centre in the vehicle frame, +x forward [m]
        MarkerOffset = [0 0]
        % Added to the marker's +x axis to get the vehicle heading [rad]
        YawOffset = 0
        % ArUco family, "" = auto-detect at lock()
        Family = ""
    end

    properties (SetAccess = private)
        % webcam object
        Cam        = []
        % cameraIntrinsics from Calib
        Intrinsics = []
        % World (floor) -> camera extrinsics
        Extr       = []
        % The same, lifted to MarkerHeight
        ExtrH      = []
        % Corner reference points in the image [px], CornerIds order
        CornerPix  = []
        Locked     = false
        % Most recent camera frame
        LastFrame  = []
    end

    methods
        function open(obj)
            obj.loadCalibration();
            if isempty(webcamlist)
                error('ArucoTracker:noCamera', ...
                    'No camera found. Check it is plugged in and not held by another program.');
            end
            obj.Cam = openCamera(obj.CamName, obj.Resolution, true);
            obj.checkIntrinsicsMatchFrame();
        end

        function close(obj)
        % Detach from the camera; openCamera keeps the device streaming.
            obj.Cam    = [];
            obj.Locked = false;
        end

        function delete(obj)
            obj.close();
        end

        function I = snap(obj)
            if isempty(obj.Cam)
                error('ArucoTracker:notOpen', 'Call open() first.');
            end
            for k = 1:3
                try
                    I = snapshot(obj.Cam);
                    break;
                catch e
                    if k == 3
                        error('ArucoTracker:frameFailed', ...
                              'Camera stopped delivering frames: %s', ...
                              strtrim(firstLineOf(e.message)));
                    end
                    pause(0.05);
                end
            end
            obj.LastFrame = I;
        end

        function ok = lock(obj, tries)
        % Find the four corner markers and build the world frame (once).
            if nargin < 2, tries = 20; end
            ok  = false;
            ids = [];
            for attempt = 1:tries
                I = obj.snap();
                [ids, ok] = obj.tryLock(I);
                if ok, break; end
                if mod(attempt, 5) == 0
                    fprintf('  lock %d/%d: saw %s, need %s\n', attempt, tries, ...
                            mat2str(unique(ids(:))'), mat2str(obj.CornerIds));
                end
            end

            if ~ok
                missing = obj.CornerIds(~ismember(obj.CornerIds, ids));
                error('ArucoTracker:noCorners', ...
                    ['Corner marker(s) %s never appeared in %d frames (saw ids: %s).\n' ...
                     'Check all four are in view, flat and not glared out.'], ...
                    mat2str(missing), tries, mat2str(unique(ids(:))'));
            end

            obj.Locked = true;
            obj.reportFrame();
        end

        function s = read(obj, markerIds, I)
        % Pose of each requested marker. s(k) has fields id, seen, xy (axle
        % centre), theta (heading, 0 = world +x), xyMarker, thetaMarker
        % (before the offsets), corners (world) and pixels (image).
        % An unseen marker returns seen = false with NaNs.
            if ~obj.Locked
                error('ArucoTracker:notLocked', 'Call lock() first.');
            end
            if nargin < 3, I = obj.snap(); end

            markerIds = markerIds(:)';
            [ids, loc] = obj.detect(I);

            s = repmat(struct('id', 0, 'seen', false, 'xy', [NaN NaN], ...
                              'theta', NaN, 'xyMarker', [NaN NaN], ...
                              'thetaMarker', NaN, 'corners', nan(4, 2), ...
                              'pixels', nan(4, 2)), 1, numel(markerIds));

            for k = 1:numel(markerIds)
                s(k).id = markerIds(k);
                j = find(ids == markerIds(k), 1);
                if isempty(j), continue; end

                pix = loc(:, :, j);
                s(k).pixels  = pix;
                Wc = obj.toWorld(obj.undistort(pix), obj.MarkerHeight);
                s(k).corners = Wc;
                s(k).xyMarker = mean(Wc, 1);

                % Marker +x axis from the TL->TR and BL->BR edges
                ax = ((Wc(2,:) - Wc(1,:)) + (Wc(3,:) - Wc(4,:))) / 2;
                s(k).thetaMarker = atan2(ax(2), ax(1));

                th = ArucoTracker.wrapPi(s(k).thetaMarker + obj.YawOffset);
                s(k).theta = th;
                R = [cos(th) -sin(th); sin(th) cos(th)];
                s(k).xy   = s(k).xyMarker - (R * obj.MarkerOffset(:))';
                s(k).seen = true;
            end
        end

        function XY = toWorld(obj, pix, h)
        % Undistorted pixels -> world metres on the plane z = h.
            if nargin < 3, h = 0; end
            if abs(h - obj.MarkerHeight) < 1e-12 && ~isempty(obj.ExtrH)
                E = obj.ExtrH;
            else
                E = ArucoTracker.liftExtrinsics(obj.Extr, h);
            end
            XY = img2world2d(pix, E, obj.Intrinsics);
        end

        function p = undistort(obj, pix)
            p = undistortPoints(pix, obj.Intrinsics);
        end

        function [ids, loc, fam] = detect(obj, I)
            if obj.Family == ""
                [ids, loc, fam] = readArucoMarker(I, 'RefineCorners', true);
            else
                [ids, loc, fam] = readArucoMarker(I, char(obj.Family), 'RefineCorners', true);
            end
        end

        function c = areaCorners(obj)
        % The arena outline, closed, for plotting.
            w = obj.worldCorners();
            c = [w; w(1, :)];
        end

        function w = worldCorners(obj)
        % The four corner reference points in world metres, CornerIds order.
            W = obj.AreaSize(1); H = obj.AreaSize(2);
            w = [0 0; W 0; W H; 0 H];
        end
    end

    methods (Access = private)
        function [ids, ok] = tryLock(obj, I)
            ok = false;
            [ids, loc, fam] = obj.detect(I);

            if obj.Family == "" && ~isempty(fam)
                obj.Family = string(fam(1));
                fprintf('family auto-detected: %s\n', obj.Family);
            end

            if ~all(ismember(obj.CornerIds, ids)), return; end

            pix = zeros(4, 2);
            for k = 1:4
                j = find(ids == obj.CornerIds(k), 1);
                pix(k, :) = obj.innerCorner(loc(:, :, j), ids, loc);
            end
            obj.CornerPix = obj.undistort(pix);
            obj.buildFrame();
            ok = true;
        end

        function loadCalibration(obj)
            f = string(obj.Calib);
            if ~isfile(f)
                error('ArucoTracker:noCalib', ...
                      '%s does not exist. Run camera_calibrate.m to make it.', f);
            end
            S  = load(f);
            fn = fieldnames(S);
            c  = [];
            for i = 1:numel(fn)
                v = S.(fn{i});
                if isa(v, 'cameraParameters') || isa(v, 'cameraIntrinsics')
                    c = v; break;
                end
            end
            if isempty(c)
                error('ArucoTracker:badCalib', ...
                      '%s holds no cameraParameters or cameraIntrinsics.', f);
            end
            if isa(c, 'cameraParameters'), c = c.Intrinsics; end
            obj.Intrinsics = c;
        end

        function checkIntrinsicsMatchFrame(obj)
        % A calibration only holds at the resolution it was taken at.
            I  = snapshot(obj.Cam);
            sz = [size(I, 1) size(I, 2)];
            cs = obj.Intrinsics.ImageSize;
            if isempty(cs) || isequal(sz, cs), return; end
            error('ArucoTracker:sizeMismatch', ...
                ['Calibration is for %dx%d but the camera is giving %dx%d.\n' ...
                 'Set Resolution = "%dx%d" or re-run camera_calibrate.m.'], ...
                cs(2), cs(1), sz(2), sz(1), cs(2), cs(1));
        end

        function p = innerCorner(obj, quad, allIds, allLoc)
        % The corner of this marker nearest the middle of the four markers.
            mids = zeros(0, 2);
            for k = 1:numel(obj.CornerIds)
                j = find(allIds == obj.CornerIds(k), 1);
                if ~isempty(j)
                    mids(end+1, :) = mean(allLoc(:, :, j), 1); %#ok<AGROW>
                end
            end
            c = mean(mids, 1);
            [~, i] = min(vecnorm(quad - c, 2, 2));
            p = quad(i, :);
        end

        function buildFrame(obj)
            worldC = obj.worldCorners();
            obj.checkCornerOrder();
            obj.Extr  = estimateExtrinsics(obj.CornerPix, worldC, obj.Intrinsics);
            obj.ExtrH = ArucoTracker.liftExtrinsics(obj.Extr, obj.MarkerHeight);
            obj.checkFrameSanity(worldC);
        end

        function checkFrameSanity(obj, worldC)
        % Warn if the locked frame cannot be the rectangle AreaSize describes.
            got = obj.toWorld(obj.CornerPix, 0);
            res = 1000 * max(vecnorm(got - worldC, 2, 2));

            d   = @(a, b) norm(obj.CornerPix(a,:) - obj.CornerPix(b,:));
            sx  = d(1,2) / (1000 * obj.AreaSize(1));
            sy  = d(1,4) / (1000 * obj.AreaSize(2));
            rat = max(sx, sy) / max(eps, min(sx, sy));

            if res > obj.FrameTolMM || rat > 1.6
                warning('ArucoTracker:frameImplausible', ...
                  ['The world frame does not match the rig: corner residual %.0f mm, ' ...
                   'W/H pixel scales differ by %.1fx.\nCheck AreaSize (%.0f x %.0f mm, ' ...
                   'inner corner to inner corner) and the CornerIds order %s.'], ...
                   res, rat, 1000*obj.AreaSize(1), 1000*obj.AreaSize(2), ...
                   mat2str(obj.CornerIds));
            end
        end

        function checkCornerOrder(obj)
        % Walking the corners in CornerIds order must not trace a bow-tie.
            if ArucoTracker.selfIntersects(obj.CornerPix)
                error('ArucoTracker:cornerOrder', ...
                    ['Corner markers %s trace a self-intersecting quad: CornerIds ' ...
                     'must list (0,0), (W,0), (W,H), (0,H) in order.'], ...
                     mat2str(obj.CornerIds));
            end
        end

        function reportFrame(obj)
            W = obj.AreaSize(1); H = obj.AreaSize(2);
            camPos = -obj.Extr.R' * obj.Extr.Translation(:);
            fprintf('world frame locked (%s):\n', obj.Family);
            fprintf('  area          %.0f x %.0f mm\n', 1000*W, 1000*H);
            fprintf('  camera at     x %.0f, y %.0f mm, %.0f mm above the floor\n', ...
                    1000*camPos(1), 1000*camPos(2), 1000*abs(camPos(3)));
            fprintf('  marker plane  %.0f mm up\n', 1000*obj.MarkerHeight);
        end
    end

    methods (Static)
        function E = liftExtrinsics(E0, h)
        % Raise the reference plane by h towards the camera, keeping the axes.
        % The sign of the camera centre's z tells which way is up.
            if h == 0, E = E0; return; end
            R = E0.R;
            C = -R' * E0.Translation(:);
            if C(3) == 0
                error('ArucoTracker:degenerateFrame', ...
                      'Camera centre lies in the floor plane - frame is degenerate.');
            end
            up = sign(C(3));
            E  = rigidtform3d(R, (E0.Translation(:) + R * [0; 0; up * h])');
        end

        function tf = selfIntersects(p)
            tf = ArucoTracker.segCross(p(1,:), p(2,:), p(3,:), p(4,:)) || ...
                 ArucoTracker.segCross(p(2,:), p(3,:), p(4,:), p(1,:));
        end

        function tf = segCross(a, b, c, d)
            o  = @(p, q, r) sign((q(1)-p(1))*(r(2)-p(2)) - ...
                                 (q(2)-p(2))*(r(1)-p(1)));
            tf = o(a,b,c) ~= o(a,b,d) && o(c,d,a) ~= o(c,d,b);
        end

        function a = wrapPi(a)
            a = mod(a + pi, 2*pi) - pi;
        end
    end
end


function s = firstLineOf(m)
    if isempty(m), s = '(no message)'; return; end
    p = split(string(m), newline);
    s = char(p(1));
end
