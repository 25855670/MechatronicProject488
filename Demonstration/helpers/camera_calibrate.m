%% Camera calibration - captures checkerboard views from the overhead camera, fits the
%% lens intrinsics and saves them to camera_params.mat in the Demonstration folder.
%% A view is kept automatically when the board is still and the view is new; Q stops early.
helpersDir = fileparts(mfilename('fullpath'));
addpath(helpersDir);

%% ================= CAMERA SETTINGS =================
% Camera name, or a substring of it
CAM_NAME = "USB";
% Capture resolution - must match the resolution used for tracking [px]
RESOLUTION = "1920x1080";

%% ================= CALIBRATION SETTINGS =================
% Checkerboard square size, measured on the print [mm]
SQUARE_MM = 60.0;
% Number of views to capture
N_WANTED = 18;
% Number of radial distortion coefficients
NUM_RADIAL = 2;
% Also estimate tangential distortion
TANGENTIAL = true;

%% ================= AUTO-CAPTURE SETTINGS =================
% Board-normal change that makes a view new [deg]
AUTO_ANG = 10;
% Board-centre movement that makes a view new [px]
AUTO_POS = 170;
% Largest mean corner motion between frames for the board to count as still [px]
AUTO_STILL = 3.0;
% Minimum time between kept views [s]
AUTO_GAP = 0.35;

%% ================= FILES =================
% Calibration output, in the Demonstration folder
OUT_FILE = fullfile(helpersDir, '..', 'camera_params.mat');
% Folder the captured views are saved to
IMG_DIR = fullfile(helpersDir, '..', 'calib_images');

%% ================= CAPTURE =================
cam = openCamera(CAM_NAME, RESOLUTION);

% Move views from a previous session aside so they do not join this fit
if ~isfolder(IMG_DIR), mkdir(IMG_DIR); end
if ~isempty(dir(fullfile(IMG_DIR, 'calib_*.png')))
    prevDir = fullfile(IMG_DIR, 'previous');
    if ~isfolder(prevDir), mkdir(prevDir); end
    delete(fullfile(prevDir, 'calib_*.png'));
    movefile(fullfile(IMG_DIR, 'calib_*.png'), prevDir);
end

fig = figure('Name', 'calibration capture', 'Color', 'w', 'NumberTitle', 'off');
setappdata(fig, 'key', '');
fig.KeyPressFcn = @(src, evt) setappdata(src, 'key', evt.Key);
fprintf('Move the camera so the board is seen tilted and in every part of the frame. Q stops early.\n');

% Rough intrinsics (focal length = image width, centred), only used to compare views
probe  = snapshot(cam);
imSz   = [size(probe, 1) size(probe, 2)];
Kguess = cameraIntrinsics([max(imSz) max(imSz)], [imSz(2)/2 imSz(1)/2], imSz);

nGot    = 0;
keptN   = zeros(0, 3);   % board normals of the kept views
keptC   = zeros(0, 2);   % board centres of the kept views [px]
allPts  = zeros(0, 2);   % every kept board corner [px]
prevPts = [];
spread  = 0;             % largest angle between two kept board normals [deg]
tLast   = -inf;
tCap    = tic;

while nGot < N_WANTED
    I = snapshot(cam);
    [pts, bs] = detectCheckerboardPoints(I, 'PartialDetections', false);
    ok = ~isempty(pts) && all(bs > 2);

    % Is the board still, and does the view differ from every kept one?
    still = false; novel = false; ang = NaN;
    if ok
        still = isequal(size(prevPts), size(pts)) && ...
                mean(vecnorm(pts - prevPts, 2, 2)) < AUTO_STILL;
        prevPts = pts;
        try
            ex  = estimateExtrinsics(pts, generateCheckerboardPoints(bs, SQUARE_MM), Kguess);
            nrm = ex.R(:, 3)';
            ctr = mean(pts, 1);
            [novel, ang] = isNovelView(nrm, ctr, keptN, keptC, AUTO_ANG, AUTO_POS);
        catch
        end
    else
        prevPts = [];
    end

    if ok && still && novel && toc(tCap) - tLast > AUTO_GAP
        nGot = nGot + 1;
        imwrite(I, fullfile(IMG_DIR, sprintf('calib_%02d.png', nGot)));
        keptN(end+1, :) = nrm; %#ok<SAGROW>
        keptC(end+1, :) = ctr; %#ok<SAGROW>
        allPts  = [allPts; pts]; %#ok<AGROW>
        tLast   = toc(tCap);
        prevPts = [];
        spread  = max(acosd(min(1, abs(keptN * keptN'))), [], 'all');
        fprintf('  kept %2d/%d   spread %3.0f deg   coverage %3.0f%%\n', ...
                nGot, N_WANTED, spread, 100*coverageFraction(allPts, imSz));
    end

    % Live view: detected corners in green, corners of kept views in orange
    imshow(I); hold on;
    if ok, plot(pts(:,1), pts(:,2), 'g.', 'MarkerSize', 8); end
    if ~isempty(allPts)
        plot(allPts(:,1), allPts(:,2), '.', 'Color', [1 .55 0], 'MarkerSize', 2);
    end
    if ~ok
        msg = 'no board';
    elseif ~still
        msg = 'hold still';
    elseif ~novel
        msg = sprintf('too similar to a kept view (%.0f deg) - move or tilt', ang);
    else
        msg = 'keeping';
    end
    title({sprintf('%d/%d kept   orientation spread %.0f deg   coverage %.0f%%', ...
                   nGot, N_WANTED, spread, 100*coverageFraction(allPts, imSz)), msg});
    hold off; drawnow;

    if ~isvalid(fig) || strcmp(getappdata(fig, 'key'), 'q'), break; end
end
if isvalid(fig), close(fig); end

%% ================= FIT =================
files = dir(fullfile(IMG_DIR, 'calib_*.png'));
if numel(files) < 6
    error('camera_calibrate:tooFew', 'Only %d view(s) captured; at least 6 are needed.', numel(files));
end
paths = fullfile(IMG_DIR, {files.name});
[imgPts, boardSize, used] = detectCheckerboardPoints(paths, 'PartialDetections', false);
if sum(used) < 6
    error('camera_calibrate:tooFew', 'Board found in only %d view(s); at least 6 are needed.', sum(used));
end

I1 = imread(paths{find(used, 1)});
cameraParams = estimateCameraParameters(imgPts, generateCheckerboardPoints(boardSize, SQUARE_MM), ...
    'ImageSize', [size(I1, 1) size(I1, 2)], ...
    'EstimateTangentialDistortion', TANGENTIAL, ...
    'NumRadialDistortionCoefficients', NUM_RADIAL, ...
    'WorldUnits', 'millimeters');

%% ================= RESULT =================
K = cameraParams.Intrinsics;
fprintf('\nviews used         %d of %d\n', sum(used), numel(paths));
fprintf('focal length       [%.1f %.1f] px\n', K.FocalLength);
fprintf('principal point    [%.1f %.1f] px\n', K.PrincipalPoint);
fprintf('radial distortion  %s\n', mat2str(K.RadialDistortion, 4));
fprintf('tangential         %s\n', mat2str(K.TangentialDistortion, 4));
fprintf('mean reproj error  %.3f px\n', cameraParams.MeanReprojectionError);
if cameraParams.MeanReprojectionError > 1.0
    warning('camera_calibrate:highError', ...
        'Mean reprojection error %.2f px is high - check the board is flat and the views are sharp.', ...
        cameraParams.MeanReprojectionError);
end

save(OUT_FILE, 'cameraParams');
fprintf('saved %s\n', OUT_FILE);

% Visual check: straight edges in the scene should come out straight
figure('Name', 'calibration check', 'Color', 'w');
subplot(1, 2, 1); imshow(I1); title('original');
subplot(1, 2, 2); imshow(undistortImage(I1, K)); title('undistorted');

%% ================= LOCAL FUNCTIONS =================
function [novel, minAng] = isNovelView(nrm, ctr, keptN, keptC, angTol, posTol)
% A view is new if its board normal is more than angTol [deg] from every kept
% normal, or its centre is more than posTol [px] from every kept centre.
    if isempty(keptN)
        novel = true; minAng = Inf; return;
    end
    minAng = min(acosd(min(1, abs(keptN * nrm(:)))));
    minPos = min(vecnorm(keptC - ctr, 2, 2));
    novel  = minAng > angTol || minPos > posTol;
end

function f = coverageFraction(pts, imSize)
% Fraction of a coarse 12 x 8 grid over the image that holds at least one corner.
    nx = 12; ny = 8;
    ix = min(nx, max(1, ceil(pts(:,1) / imSize(2) * nx)));
    iy = min(ny, max(1, ceil(pts(:,2) / imSize(1) * ny)));
    f  = numel(unique(sub2ind([ny nx], iy, ix))) / (nx * ny);
end
