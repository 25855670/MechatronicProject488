classdef FleetLink < handle
%FLEETLINK  Bluetooth link to a fleet of DualWheelDriveBT vehicles.
%
%   f = FleetLink(["VEH1" "VEH2" "VEH3"]);   % paired HC-06 names (or COMn)
%   f.connect();
%   f.stepAll([12 12; 10 14; 8 8]);          % [wL wR] per vehicle [rad/s]
%   f.telemetry();                           % drain telemetry into f.Log
%   f.close();

    properties (SetAccess = private)
        % Device names or COM ports, as given
        Names    string
        % bluetooth / serialport objects
        Dev      cell
        % Per-link state: the port is open
        Open     logical
        % The vehicle id was set and read back correctly
        Confirmed logical
    end

    properties
        % Vehicle id per link, 1..9 (labels telemetry column 1)
        Ids double
        % Wheel PI gains pushed at connect, one row per vehicle [kp ki]
        Gains double = []
        % Telemetry log [id tMs spL wL uL spR wR uR flags tHost], one row per line
        Log double = []
        % tic that stamps telemetry column 10 (tHost)
        Clock uint64 = uint64(0)
    end

    properties (Access = private)
        % Partial-line carry per link
        RxBuf   string
    end

    properties (Constant)
        % Serial baud (ignored by SPP, must match the HC-06 UART)
        BAUD            = 115200
        % Read timeout [s]
        TIMEOUT         = 1.0
        % Time to wait for the radio link to come up [s]
        CONNECT_TIMEOUT = 6.0
        % Open attempts per vehicle
        OPEN_RETRIES    = 3
        % Pause between open attempts [s]
        OPEN_RETRY_S    = 1.5
        % Pause between opening successive vehicles [s]
        OPEN_STAGGER_S  = 1.0
    end

    methods
        function obj = FleetLink(names)
            obj.Names = string(names(:))';
            n = numel(obj.Names);
            obj.Dev   = cell(1, n);
            obj.Ids   = 1:n;
            obj.Open  = false(1, n);
            obj.Confirmed = false(1, n);
            obj.RxBuf = repmat("", 1, n);
        end

        function set.Ids(obj, v)
            v = v(:)';
            if ~isnumeric(v) || any(v < 1) || any(v > 9) || any(mod(v, 1) ~= 0)
                error('FleetLink:badId', 'Ids must be whole numbers in 1..9.');
            end
            if ~isempty(obj.Names) && numel(v) ~= numel(obj.Names) %#ok<MCSUP>
                error('FleetLink:badId', 'Need %d ids, got %d.', ...
                      numel(obj.Names), numel(v));                %#ok<MCSUP>
            end
            obj.Ids = v;
        end

        function connect(obj)
        % Open every link (staggered, with retries), confirm ids, push gains.
            for k = 1:numel(obj.Names)
                if k > 1, pause(obj.OPEN_STAGGER_S); end
                obj.openOne(k);
            end
            obj.pushGains();

            fprintf('--- links ---\n');
            for k = 1:numel(obj.Names)
                if obj.Confirmed(k)
                    st = sprintf('id %d confirmed', obj.Ids(k));
                elseif obj.Open(k)
                    st = 'port open but id unconfirmed';
                else
                    st = 'NOT OPEN';
                end
                fprintf('  %-6s %s\n', obj.Names(k), st);
            end
        end

        function stepAll(obj, W)
        % W is Nx2 of [wL wR] in rad/s, one row per vehicle.
            if size(W, 1) ~= numel(obj.Names) || size(W, 2) ~= 2
                error('FleetLink:badStep', ...
                      'W must be %dx2 of [wL wR].', numel(obj.Names));
            end
            for k = 1:numel(obj.Names)
                obj.send(k, sprintf('S%.3f:%.3f', W(k, 1), W(k, 2)));
            end
        end

        function setTelemetry(obj, ms)
        % Telemetry period in ms for every vehicle, 0 = off.
            for k = 1:numel(obj.Names)
                obj.send(k, sprintf('T%d', round(ms)));
            end
        end

        function heartbeat(obj)
        % Feed the vehicles' link watchdogs without changing their setpoints.
            for k = 1:numel(obj.Names)
                obj.send(k, 'h');
            end
        end

        function stopAll(obj)
        % Stop every vehicle, best effort per link.
            for k = 1:numel(obj.Names)
                try
                    obj.send(k, 'x');
                catch
                end
            end
        end

        function telemetry(obj)
        % Drain every link without blocking and append parsed rows to Log.
            if obj.Clock == 0, obj.Clock = tic; end
            tHost = toc(obj.Clock);

            for k = 1:numel(obj.Names)
                if ~obj.Open(k), continue; end

                try
                    nb = obj.Dev{k}.NumBytesAvailable;
                catch
                    obj.Open(k) = false;
                    continue;
                end
                if nb > 0
                    obj.RxBuf(k) = obj.RxBuf(k) + ...
                        string(read(obj.Dev{k}, nb, 'char'));
                end

                lines = split(obj.RxBuf(k), newline);
                obj.RxBuf(k) = lines(end);
                for i = 1:numel(lines) - 1
                    row = FleetLink.parseTelemetry(lines(i));
                    if ~isempty(row)
                        obj.Log = [obj.Log; row, tHost];
                    else
                        s = strtrim(lines(i));
                        if strlength(s) > 0
                            fprintf('  %s: %s\n', obj.Names(k), s);
                        end
                    end
                end
            end
        end

        function close(obj)
            obj.stopAll();
            for k = 1:numel(obj.Names)
                if obj.Open(k)
                    obj.Dev{k}  = [];
                    obj.Open(k) = false;
                end
            end
        end

        function delete(obj)
            obj.close();
        end
    end

    methods (Access = private)
        function openOne(obj, k)
            name = obj.Names(k);
            lastErr = '';
            for attempt = 1:obj.OPEN_RETRIES
                try
                    if ~isempty(regexp(name, '^COM\d+$', 'once'))
                        d = serialport(name, obj.BAUD);
                    else
                        d = bluetooth(name);
                    end
                    d.Timeout = obj.TIMEOUT;
                    configureTerminator(d, "LF");
                    obj.Dev{k}  = d;
                    obj.Open(k) = true;
                    break;
                catch e
                    lastErr = e.message;
                    if attempt < obj.OPEN_RETRIES
                        fprintf('%s: open attempt %d/%d failed, retrying...\n', ...
                                name, attempt, obj.OPEN_RETRIES);
                        pause(obj.OPEN_RETRY_S);
                    end
                end
            end

            if ~obj.Open(k)
                warning('FleetLink:openFailed', ...
                    ['%s did not open after %d attempts: %s\n' ...
                     'Check the module is powered and in range, and that no ' ...
                     'other program or old FleetLink object holds the port.'], ...
                    name, obj.OPEN_RETRIES, lastErr);
                return;
            end

            obj.Confirmed(k) = obj.handshake(k);
        end

        function n = bytesWaiting(obj, k)
            try
                n = obj.Dev{k}.NumBytesAvailable;
            catch
                n = 0;
            end
        end

        function s = drainRaw(obj, k)
            s = "";
            n = obj.bytesWaiting(k);
            if n > 0
                try
                    s = string(read(obj.Dev{k}, n, 'char'));
                catch
                end
            end
        end

        function ok = handshake(obj, k)
        % Wake the link, set the vehicle id, then read it back from the
        % identity line "# BT v<id> ...".
            ok   = false;
            want = obj.Ids(k);
            name = obj.Names(k);

            t0 = tic;
            while toc(t0) < obj.CONNECT_TIMEOUT
                obj.send(k, '?');
                pause(0.25);
                if obj.bytesWaiting(k) > 0, break; end
            end
            if obj.bytesWaiting(k) == 0
                warning('FleetLink:noLink', ...
                    '%s opened but said nothing in %.0f s.', name, obj.CONNECT_TIMEOUT);
                return;
            end

            flush(obj.Dev{k});
            obj.send(k, sprintf('V%d', want));
            pause(0.2);

            flush(obj.Dev{k});
            obj.send(k, '?');
            pause(0.4);
            txt = obj.drainRaw(k);

            tok = regexp(txt, '#\s*BT\s+v(\d+)', 'tokens', 'once');
            if isempty(tok)
                warning('FleetLink:noIdentity', ...
                    '%s: link is up but no identity line came back.', name);
                return;
            end

            got = str2double(tok{1});
            if got ~= want
                warning('FleetLink:idMismatch', ...
                    '%s: asked for id %d, vehicle reports %d.', name, want, got);
                return;
            end

            ok = true;
            fprintf('%s: link up, id %d confirmed\n', name, got);
        end

        function pushGains(obj)
            if isempty(obj.Gains), return; end
            if size(obj.Gains, 1) ~= numel(obj.Names) || size(obj.Gains, 2) ~= 2
                error('FleetLink:badGains', 'Gains must be %dx2 [kp ki].', ...
                      numel(obj.Names));
            end
            for k = 1:numel(obj.Names)
                obj.send(k, sprintf('KP%.6g', obj.Gains(k, 1)));
                obj.send(k, sprintf('KI%.6g', obj.Gains(k, 2)));
            end
            pause(0.3);
        end

        function send(obj, k, cmd)
            if ~obj.Open(k), return; end
            try
                writeline(obj.Dev{k}, cmd);
            catch e
                obj.Open(k) = false;
                warning('FleetLink:writeFailed', ...
                        '%s dropped: %s', obj.Names(k), e.message);
            end
        end
    end

    methods (Static)
        function row = parseTelemetry(line)
        % One line -> a 1x9 telemetry row, or [] ('#' lines are messages).
            row = [];
            s = strtrim(line);
            if strlength(s) == 0 || startsWith(s, "#"), return; end
            v = str2double(split(s, ","));
            if numel(v) == 9 && ~any(isnan(v))
                row = v(:)';
            end
        end

        function W = wheelSpeeds(v, omega, r, b)
        % Body (v [m/s], omega [rad/s], +ve left) -> [wL wR] [rad/s].
        % The drivetrain is forward only, so negative wheel speeds clamp to 0.
            wL = (v - omega .* b / 2) ./ r;
            wR = (v + omega .* b / 2) ./ r;
            W = [max(wL(:), 0) max(wR(:), 0)];
        end
    end
end
