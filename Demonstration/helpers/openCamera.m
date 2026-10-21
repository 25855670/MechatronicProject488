function cam = openCamera(name, resolution, verbose)
%OPENCAMERA  Open the USB camera once and keep it streaming between runs.
%
%   cam = openCamera("USB", "1920x1080")
%   openCamera release        % close it so other programs can use it
%
%   Closing and reopening the camera back-to-back makes it stop returning
%   frames, so the open camera is cached and reused by later calls.

    persistent CAM CAM_NAME

    if nargin >= 1 && (isstring(name) || ischar(name)) && lower(string(name)) == "release"
        if ~isempty(CAM)
            try
                delete(CAM);
            catch
            end
        end
        CAM = []; CAM_NAME = "";
        fprintf('camera released.\n');
        if nargout > 0, cam = []; end
        return;
    end

    if nargin < 2, resolution = ""; end
    if nargin < 3, verbose = true; end

    % Open attempts before giving up
    OPEN_TRIES = 4;
    % Wait before each retry [s]
    WAITS      = [1.5 3.0 5.0];
    % Time to let auto-exposure settle after opening [s]
    WARM_S     = 2.0;
    % Consecutive good frames needed after settling
    WARM_GOOD  = 5;

    %% ================= REUSE THE CACHED CAMERA =================
    if ~isempty(CAM) && CAM_NAME == string(name)
        try
            live = string(CAM.Resolution);
            want = string(resolution);
            if strlength(want) > 0 && live ~= want
                if verbose
                    fprintf('camera: retuning %s -> %s in place\n', live, want);
                end
                CAM.Resolution = char(want);
                ok = warmUp(CAM, WARM_S, WARM_GOOD);
            else
                snapshot(CAM);
                ok = true;
            end
            if ok
                cam = CAM;
                if verbose
                    fprintf('camera: reusing the open "%s" @ %s\n', CAM_NAME, CAM.Resolution);
                end
                return;
            end
        catch
        end
        if verbose
            fprintf('cached camera stopped delivering - reopening\n');
        end
    end
    if ~isempty(CAM)
        try
            delete(CAM);
        catch
        end
        CAM = [];
        pause(1.0);
    end

    %% ================= COLD OPEN WITH RETRIES =================
    list = webcamlist;
    if isempty(list)
        error('openCamera:noCamera', 'webcamlist is empty - no camera is available.');
    end
    sel = pickOne(name, list, verbose);

    lastErr = '';
    c       = [];
    misses  = 0;
    for attempt = 1:OPEN_TRIES
        if isempty(c)
            try
                c = webcam(sel);
            catch e
                lastErr = e.message;
                c       = [];
            end
            if ~isempty(c) && strlength(string(resolution)) > 0
                try
                    c.Resolution = char(resolution);
                catch
                    warning('openCamera:resolution', ...
                        'Camera refused %s; left at %s. Available: %s', ...
                        resolution, c.Resolution, ...
                        strjoin(string(c.AvailableResolutions), ', '));
                end
            end
            misses = 0;
        end

        if ~isempty(c)
            [ok, m] = warmUp(c, WARM_S, WARM_GOOD);
            if ~isempty(m), lastErr = m; end
            if ok
                CAM      = c;
                CAM_NAME = string(name);
                cam      = CAM;
                if verbose
                    fprintf('camera: opened "%s" @ %s\n', c.Name, c.Resolution);
                end
                return;
            end
            % Give the same handle a second chance before reopening it
            misses = misses + 1;
            if misses >= 2
                try
                    delete(c);
                catch
                end
                c = [];
            end
        end

        if attempt < OPEN_TRIES
            w = WAITS(min(attempt, numel(WAITS)));
            if verbose
                fprintf('camera did not start (%s), retrying in %.1f s (%d of %d)\n', ...
                        strtrim(firstLine(lastErr)), w, attempt + 1, OPEN_TRIES);
            end
            pause(w);
        end
    end

    try
        delete(c);
    catch
    end
    error('openCamera:wontStart', ...
        ['"%s" never delivered frames after %d attempts (last error: %s).\n' ...
         'Check no other program holds the camera and that it is plugged ' ...
         'straight into the computer.'], ...
        string(sel), OPEN_TRIES, strtrim(firstLine(lastErr)));
end


function sel = pickOne(name, list, verbose)
% Camera matching name; with no name, the last one listed (not the laptop's).
    list = string(list(:)');
    if isnumeric(name) && ~isempty(name)
        sel = name; return;
    end
    if strlength(string(name)) == 0
        sel = char(list(end));
        if numel(list) > 1 && verbose
            fprintf('no camera name given, taking "%s"\n', sel);
        end
        return;
    end
    hit = find(contains(lower(list), lower(string(name))), 1);
    if isempty(hit)
        error('openCamera:noSuchCamera', ...
              'No camera matching "%s". Available: %s', name, strjoin(list, ', '));
    end
    sel = char(list(hit));
end


function s = firstLine(m)
    if isempty(m), s = '(no message)'; return; end
    p = split(string(m), newline);
    s = char(p(1));
end


function [ok, msg] = warmUp(c, warmS, needGood)
% Let the stream settle for warmS seconds, then require needGood
% consecutive frames.
    msg = '';
    tW = tic;
    while toc(tW) < warmS
        try
            snapshot(c);
        catch e
            msg = e.message;
            pause(0.05);
        end
    end

    good = 0;
    for i = 1:(3 * needGood)
        try
            snapshot(c);
            good = good + 1;
        catch e
            msg  = e.message;
            good = 0;
        end
        if good >= needGood, break; end
    end
    ok = good >= needGood;
end
