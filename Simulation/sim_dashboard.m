function fig = sim_dashboard(out, WP, opts)
%SIM_DASHBOARD  Dashboard of a PurePursuitControlledRobot simulation, in the
%   same layout and style as the hardware run dashboard.
%
%   fig = sim_dashboard(out, {waypoints_v1, waypoints_v2, waypoints_v3}, opts)
%
%   out   the Simulink.SimulationOutput returned by sim
%   WP    commanded waypoints per vehicle, each N x 2 [m]
%   opts  struct with fields
%     .area       movement area corners [xmin ymin; xmax ymax] [m]
%     .cellSize   planner grid cell size [m]
%     .safeDist   centre-to-centre separation of concern [m]
%     .R_wheel    wheel radius [m]
%     .L_track    track width [m]
%     .lookahead  pure pursuit lookahead distance [m]

VEH = ["Vehicle 1" "Vehicle 2" "Vehicle 3"];

COL = [0.00 0.45 0.74;      % v1 blue
       0.78 0.10 0.12;      % v2 red
       0.20 0.60 0.25];     % v3 green

dk = @(c) max(c*0.60, 0);            % dark shade
lt = @(c) c + (1-c)*0.58;            % light shade

MAXPT = 4000;               % decimate for plotting
ARENA = [opts.area(1,1) opts.area(2,1) opts.area(1,2) opts.area(2,2)];

ls   = out.logsout;
tMax = out.tout(end);

%% ----------------------------------------------------------- figure --
fig = figure("Color","w","Position",[60 40 1560 980]);
tl  = tiledlayout(fig, 12, 4, "TileSpacing","compact", "Padding","compact");
title(tl, "Simulation run", "FontWeight","bold", "FontSize",14, "Interpreter","none");

%% ---------------------------------------------------------------- map --
axMap = nexttile(tl, 1, [8 2]);
hold(axMap, "on")

rectangle(axMap, "Position", [ARENA(1) ARENA(3) ...
                              ARENA(2)-ARENA(1) ARENA(4)-ARENA(3)], ...
          "EdgeColor", [0.25 0.25 0.25], "LineWidth", 1.4);

hLine   = gobjects(1,3);
summary = strings(1,3);
POSE    = cell(1,3);
TIME    = cell(1,3);
epS = nan(3,2); epG = nan(3,2); epC = zeros(3,3);

for k = 1:3
    [tp, P] = getSig(ls, VEH(k), "pose");
    W = WP{k};
    C = COL(k,:);
    POSE{k} = P; TIME{k} = tp;

    plot(axMap, W(:,1), W(:,2), "--", "Color", [lt(C) 0.9], "LineWidth", 1.1);
    plot(axMap, W(1,1),   W(1,2),   "o", "Color", dk(C), ...
         "MarkerFaceColor","w", "MarkerSize",7, "LineWidth",1.3);
    plot(axMap, W(end,1), W(end,2), "s", "Color", dk(C), ...
         "MarkerFaceColor", dk(C), "MarkerSize",8);
    epS(k,:) = W(1,:);
    epG(k,:) = W(end,:);
    epC(k,:) = dk(C);

    if isempty(P), continue, end

    d = dec(size(P,1), MAXPT);
    hLine(k) = plot(axMap, P(d,1), P(d,2), "-", "Color", C, "LineWidth", 1.9);

    e = zeros(numel(d),1);
    for i = 1:numel(d)
        e(i) = distToPath(P(d(i),1:2), W);
    end
    summary(k) = sprintf("XTE peak %.0f mm", max(e)*1000);
end

markRouteEnds(axMap, epS, epG, epC, opts.area);
hold(axMap, "off")
axis(axMap, "equal")
xlim(axMap, [ARENA(1) ARENA(2)] + [-0.50 0.50])   % room for the endpoint labels
ylim(axMap, [ARENA(3) ARENA(4)] + [-0.34 0.34])
xticks(axMap, ARENA(1):opts.cellSize:ARENA(2))
yticks(axMap, ARENA(3):opts.cellSize:ARENA(4))
grid(axMap, "on"), box(axMap, "off")
xlabel(axMap, "x [m]"), ylabel(axMap, "y [m]")
title(axMap, "Path following")
subtitle(axMap, "Simulated (solid) paths vs commanded (dashed)", ...
         "FontSize", 8, "Color", [0.4 0.4 0.4])
ok = isgraphics(hLine);
if any(ok)
    legend(axMap, hLine(ok), VEH(ok), "Location","southoutside", ...
           "Orientation","horizontal", "Box","off")
end

%% ------------------------------------------- inter-vehicle separation --
axSep = nexttile(tl, 33, [4 2]);
hold(axSep, "on")

if all(cellfun(@(p) ~isempty(p), POSE))
    % Each vehicle logs on its own time vector, so interpolate onto one grid
    tEnd = min(cellfun(@(t) t(end), TIME));
    tq   = linspace(0, tEnd, MAXPT).';
    XY   = cell(1,3);
    for k = 1:3
        XY{k} = [interp1(TIME{k}, POSE{k}(:,1), tq, "linear", "extrap"), ...
                 interp1(TIME{k}, POSE{k}(:,2), tq, "linear", "extrap")];
    end

    prs  = [1 2; 1 3; 2 3];
    hSep = gobjects(1,3);
    lSep = strings(1,3);
    sty  = ["-" "--" "-."];
    gry  = [0.15; 0.45; 0.70];        % dark -> light, one per pair

    for j = 1:3
        a = prs(j,1); b = prs(j,2);
        dsep = hypot(XY{a}(:,1)-XY{b}(:,1), XY{a}(:,2)-XY{b}(:,2));
        hSep(j) = plot(axSep, tq, dsep, sty(j), ...
                       "Color", gry(j)*[1 1 1], "LineWidth", 1.5);
        lSep(j) = sprintf("V%d - V%d   (min %.0f mm)", a, b, min(dsep)*1000);
    end

    yline(axSep, opts.safeDist, ":", sprintf("%.0f mm", opts.safeDist*1000), ...
          "Color",[0.75 0.20 0.20], "LineWidth",1.2, ...
          "LabelHorizontalAlignment","left", "FontSize",8);

    legend(axSep, hSep, lSep, "Location","southoutside", ...
           "Orientation","horizontal", "Box","off", "FontSize",8)
    ylim(axSep, [0 max(ylim(axSep))])
end

hold(axSep, "off")
grid(axSep, "on"), box(axSep, "on")
xlim(axSep, [0 tMax])
xlabel(axSep, "Time [s]"), ylabel(axSep, "Separation [m]")
title(axSep, "Distance between vehicles")

%% ------------------------------------------------- per-vehicle panels --
tileSpeed = [3 19 35];

for k = 1:3
    C = COL(k,:); Cd = C; Cl = lt(C);

    ax = nexttile(tl, tileSpeed(k), [4 2]);
    P = POSE{k}; tp = TIME{k};
    hh = gobjects(1,4);  ll = strings(1,4);

    % ---- linear speed ------------------------------------------------
    yyaxis(ax, "left")
    hold(ax, "on")
    [tv, wl] = getSig(ls, VEH(k), "omega_L_MT");
    [tw, wr] = getSig(ls, VEH(k), "omega_R_MT");
    if ~isempty(wl) && ~isempty(wr)
        wr = interp1(tw, wr, tv, "linear", "extrap");
        v  = opts.R_wheel*(wr + wl)/2;
        d  = dec(numel(tv), MAXPT);
        hh(1) = plot(ax, tv(d), v(d), "-", "Color", Cd, "LineWidth", 1.4);
        ll(1) = "v encoder";
    end

    % Commanded speed and turn rate from the wheel speed references
    [tr, wLr] = getSig(ls, VEH(k), "wL_ref");
    [~,  wRr] = getSig(ls, VEH(k), "wR_ref");
    haveCmd = ~isempty(wLr) && ~isempty(wRr);
    if haveCmd
        dr   = dec(numel(tr), MAXPT);
        tr   = tr(dr);
        vCmd = opts.R_wheel*(wRr(dr) + wLr(dr))/2;
        wCmd = opts.R_wheel*(wRr(dr) - wLr(dr))/opts.L_track;
        hh(2) = plot(ax, tr, vCmd, "--", "Color", Cd, "LineWidth", 1.1);
        ll(2) = "v commanded";
    end
    hold(ax, "off")
    ylabel(ax, "v [m/s]")
    ax.YAxis(1).Color = Cd;

    % ---- heading -----------------------------------------------------
    yyaxis(ax, "right")
    hold(ax, "on")
    if ~isempty(P)
        d = dec(size(P,1), MAXPT);
        hh(3) = plot(ax, tp(d), breakWraps(wrapDeg(P(d,3))), "-", "Color", Cl, "LineWidth", 1.2);
        ll(3) = "\theta simulated";

        % Commanded heading = bearing to the lookahead point, theta + alpha,
        % with alpha from omega = v*2*sin(alpha)/lookahead
        if haveCmd
            alpha = asin(max(-1, min(1, wCmd.*opts.lookahead./(2*vCmd))));
            alpha(vCmd < 0.02) = NaN;
            thCmd = interp1(tp, P(:,3), tr, "linear", "extrap") + alpha;
            hh(4) = plot(ax, tr, breakWraps(wrapDeg(thCmd)), "--", "Color", Cl, "LineWidth", 1.0);
            ll(4) = "\theta commanded";
        end
    end
    hold(ax, "off")
    ylabel(ax, "\theta [deg]")
    ax.YAxis(2).Color = Cl;
    ylim(ax, [-190 190]), yticks(ax, -180:90:180)

    xlim(ax, [0 tMax])
    grid(ax, "on"), box(ax, "on")
    title(ax, VEH(k) + "  -  speed and heading, simulated vs commanded", "Color", Cd)
    if strlength(summary(k)) > 0
        subtitle(ax, summary(k), "FontSize", 8, "Color", [0.4 0.4 0.4])
    end

    keep = isgraphics(hh);
    if any(keep)
        legend(ax, hh(keep), ll(keep), "Location","southoutside", ...
               "Orientation","horizontal", "Box","off", "FontSize",7)
    end
    if k == 3, xlabel(ax, "Time [s]"), end
end
end

%% ------------------------------------------------------------ helpers --
function [t, y] = getSig(ls, tag, name)
%GETSIG  Fetch one named signal from one model-reference instance.
t = []; y = [];
for i = 1:ls.numElements
    e = ls{i};
    if ~strcmpi(e.Name, name), continue, end
    if ~contains(string(e.BlockPath.convertToCell{1}), tag), continue, end
    d = e.Values.Data;
    if ndims(d) == 3
        % A column-vector signal logs as [k 1 N]
        d = reshape(d, [], size(d,3)).';
    elseif isrow(d)
        d = d(:);
    end
    t = e.Values.Time;
    y = d;
    return
end
end

function markRouteEnds(ax, starts, goals, cols, area)
%MARKROUTEENDS  Label every route endpoint ("VEHk Start"/"VEHk End") outside
%   the nearest arena edge; coincident points merge, neighbours alternate rank.
N = size(starts, 1);
pts = [starts; goals];
txt = cell(2*N, 1);
col = [cols; cols];
for v = 1:N
    txt{v}     = sprintf('VEH%d Start', v);
    txt{N + v} = sprintf('VEH%d End',   v);
end
keep = all(isfinite(pts), 2);
pts = pts(keep,:); txt = txt(keep); col = col(keep,:);
if isempty(pts), return; end

% Merge anything sharing a point
key = round(pts / 1e-4);
[~, ia, ic] = unique(key, 'rows', 'stable');
nG = numel(ia);
gPos = pts(ia,:); gTxt = cell(nG,1); gCol = zeros(nG,3);
for g = 1:nG
    m = find(ic == g);
    if numel(m) == 1
        gTxt{g} = txt{m};  gCol(g,:) = col(m,:);
    else
        gTxt{g} = txt(m);  gCol(g,:) = [0.15 0.15 0.15];
    end
end

% Which edge, and where along it
gEdge = zeros(nG,1); gAlong = zeros(nG,1);
for g = 1:nG
    d = [abs(gPos(g,2)-area(1,2)), abs(area(2,2)-gPos(g,2)), ...
         abs(gPos(g,1)-area(1,1)), abs(area(2,1)-gPos(g,1))];
    [~, gEdge(g)] = min(d);
    if gEdge(g) <= 2, gAlong(g) = gPos(g,1); else, gAlong(g) = gPos(g,2); end
end

% Alternate ranks where neighbours are too close to clear
gRow = zeros(nG,1);
for e = 1:4
    idx = find(gEdge == e);
    if numel(idx) < 2, continue; end
    [~, ord] = sort(gAlong(idx));
    idx = idx(ord);
    if e <= 2, need = 0.62; else, need = 0.26; end
    for q = 2:numel(idx)
        if gAlong(idx(q)) - gAlong(idx(q-1)) < need
            gRow(idx(q)) = 1 - gRow(idx(q-1));
        end
    end
end

gap = 0.07; step = 0.15;
for g = 1:nG
    d = gap + gRow(g)*step;
    switch gEdge(g)
        case 1, pos = [gPos(g,1), area(1,2)-d]; ha='center'; va='top';
        case 2, pos = [gPos(g,1), area(2,2)+d]; ha='center'; va='bottom';
        case 3, pos = [area(1,1)-d, gPos(g,2)]; ha='right';  va='middle';
        otherwise, pos = [area(2,1)+d, gPos(g,2)]; ha='left'; va='middle';
    end
    text(ax, pos(1), pos(2), gTxt{g}, ...
         'Color', gCol(g,:), 'FontSize', 9, ...
         'FontWeight', 'bold', 'HorizontalAlignment', ha, ...
         'VerticalAlignment', va, 'HandleVisibility', 'off');
end
end

function idx = dec(n, maxpt)
idx = 1:max(1, ceil(n/maxpt)):n;
end

function th = wrapDeg(theta)
th = rad2deg(theta - 2*pi*floor((theta + pi)/(2*pi)));
end

function th = breakWraps(th)
%BREAKWRAPS  Gap the line where the heading wraps between +180 and -180 deg.
th([false; abs(diff(th(:))) > 180]) = NaN;
end

function d = distToPath(p, W)
%DISTTOPATH  Perpendicular distance to the commanded polyline.
A  = W(1:end-1,:);
B  = W(2:end,:);
AB = B - A;
AP = p - A;
t  = max(0, min(1, sum(AP.*AB,2) ./ max(sum(AB.^2,2), eps)));
C  = A + t.*AB;
d  = min(hypot(C(:,1)-p(1), C(:,2)-p(2)));
end
