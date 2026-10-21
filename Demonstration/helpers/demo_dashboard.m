function fig = demo_dashboard(stem, opts)
%DEMO_DASHBOARD  Dashboard of a hardware demonstration run, in the same
%   layout and style as the simulation dashboard.
%
%   fig = demo_dashboard(stem, opts)
%
%   stem  run file stem, e.g. "runs/run_20261019_150151"; reads the
%         <stem>_VEHk_host.csv, _wheel.csv and _path.csv files the run wrote
%   opts  struct with fields
%     .area        planner area corners [xmin ymin; xmax ymax] [m]
%     .cellSize    planner grid cell size [m]
%     .safeDist    centre-to-centre separation of concern [m]
%     .planOffset  [dx dy] from mat to planner coordinates [m]

NAME = ["VEH1" "VEH2" "VEH3"];
VEH  = ["Vehicle 1" "Vehicle 2" "Vehicle 3"];

COL = [0.00 0.45 0.74;      % v1 blue
       0.78 0.10 0.12;      % v2 red
       0.20 0.60 0.25];     % v3 green

dk = @(c) max(c*0.60, 0);            % dark shade
lt = @(c) c + (1-c)*0.58;            % light shade

MAXPT = 4000;               % decimate for plotting
ARENA = [opts.area(1,1) opts.area(2,1) opts.area(1,2) opts.area(2,2)];

%% ------------------------------------------------------------- load --
H = cell(1,3); Wh = cell(1,3); WP = cell(1,3);
for k = 1:3
    H{k}  = readIf(stem + "_" + NAME(k) + "_host.csv");
    Wh{k} = readIf(stem + "_" + NAME(k) + "_wheel.csv");
    P     = readIf(stem + "_" + NAME(k) + "_path.csv");
    % Poses and paths are logged in mat coordinates; draw in planner coordinates
    if ~isempty(H{k})
        H{k}.x_m = H{k}.x_m + opts.planOffset(1);
        H{k}.y_m = H{k}.y_m + opts.planOffset(2);
    end
    if ~isempty(P)
        WP{k} = [P.x_m P.y_m] + opts.planOffset;
    end
end
have = ~cellfun(@isempty, H);
if ~any(have)
    error("demo_dashboard:noData", "No %s_VEH*_host.csv files found.", stem);
end

tMax = 0;
for k = 1:3
    if ~isempty(H{k}),  tMax = max([tMax; H{k}.t_s(:)]);  end
    if ~isempty(Wh{k}), tMax = max([tMax; Wh{k}.t_s(:)]); end
end
if ~(tMax > 0) || ~isfinite(tMax), tMax = 1; end

%% ----------------------------------------------------------- figure --
fig = figure("Color","w","Position",[60 40 1560 980]);
tl  = tiledlayout(fig, 12, 4, "TileSpacing","compact", "Padding","compact");
title(tl, "Demonstration run", "FontWeight","bold", "FontSize",14, "Interpreter","none");

%% ---------------------------------------------------------------- map --
axMap = nexttile(tl, 1, [8 2]);
hold(axMap, "on")

rectangle(axMap, "Position", [ARENA(1) ARENA(3) ...
                              ARENA(2)-ARENA(1) ARENA(4)-ARENA(3)], ...
          "EdgeColor", [0.25 0.25 0.25], "LineWidth", 1.4);

hLine   = gobjects(1,3);
summary = strings(1,3);
epS = nan(3,2); epG = nan(3,2); epC = zeros(3,3);

for k = 1:3
    if ~have(k), continue, end
    W = WP{k};
    C = COL(k,:);

    if ~isempty(W)
        plot(axMap, W(:,1), W(:,2), "--", "Color", [lt(C) 0.9], "LineWidth", 1.1);
        plot(axMap, W(1,1),   W(1,2),   "o", "Color", dk(C), ...
             "MarkerFaceColor","w", "MarkerSize",7, "LineWidth",1.3);
        plot(axMap, W(end,1), W(end,2), "s", "Color", dk(C), ...
             "MarkerFaceColor", dk(C), "MarkerSize",8);
        epS(k,:) = W(1,:);
        epG(k,:) = W(end,:);
        epC(k,:) = dk(C);
    end

    % NaN rows are dropped camera frames, left as gaps in the line
    P = [H{k}.x_m H{k}.y_m];
    d = dec(size(P,1), MAXPT);
    hLine(k) = plot(axMap, P(d,1), P(d,2), "-", "Color", C, "LineWidth", 1.9);

    e = H{k}.xte_m(~isnan(H{k}.xte_m));
    if ~isempty(e)
        summary(k) = sprintf("XTE peak %.0f mm", max(e)*1000);
    end
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
subtitle(axMap, "Tracked (solid) paths vs commanded (dashed)", ...
         "FontSize", 8, "Color", [0.4 0.4 0.4])
ok = isgraphics(hLine);
if any(ok)
    legend(axMap, hLine(ok), VEH(ok), "Location","southoutside", ...
           "Orientation","horizontal", "Box","off")
end

%% ------------------------------------------- inter-vehicle separation --
axSep = nexttile(tl, 33, [4 2]);
hold(axSep, "on")

if sum(have) >= 2
    % Vehicles are sampled at different instants, so interpolate each pair
    % onto a common grid over the time both were tracked
    prs  = nchoosek(find(have), 2);
    hSep = gobjects(1, size(prs,1));
    lSep = strings(1, size(prs,1));
    sty  = ["-" "--" "-."];
    gry  = [0.15; 0.45; 0.70];        % dark -> light, one per pair

    for j = 1:size(prs,1)
        a = prs(j,1); b = prs(j,2);
        [ta, xa, ya] = seenXY(H{a});
        [tb, xb, yb] = seenXY(H{b});
        if numel(ta) < 2 || numel(tb) < 2, continue, end
        t0 = max(min(ta), min(tb));
        t1 = min(max(ta), max(tb));
        if t1 <= t0, continue, end
        tq   = linspace(t0, t1, MAXPT).';
        dsep = hypot(interp1(ta,xa,tq) - interp1(tb,xb,tq), ...
                     interp1(ta,ya,tq) - interp1(tb,yb,tq));
        hSep(j) = plot(axSep, tq, dsep, sty(j), ...
                       "Color", gry(j)*[1 1 1], "LineWidth", 1.5);
        lSep(j) = sprintf("V%d - V%d   (min %.0f mm)", a, b, min(dsep)*1000);
    end

    yline(axSep, opts.safeDist, ":", sprintf("%.0f mm", opts.safeDist*1000), ...
          "Color",[0.75 0.20 0.20], "LineWidth",1.2, ...
          "LabelHorizontalAlignment","left", "FontSize",8);

    keep = isgraphics(hSep);
    if any(keep)
        legend(axSep, hSep(keep), lSep(keep), "Location","southoutside", ...
               "Orientation","horizontal", "Box","off", "FontSize",8)
        ylim(axSep, [0 max(ylim(axSep))])
    end
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
    if ~have(k)
        axis(ax, "off");
        text(ax, 0.5, 0.5, VEH(k) + " not in this run", "Units","normalized", ...
             "HorizontalAlignment","center", "Color",[0.6 0.6 0.6]);
        continue
    end

    h  = H{k};
    d  = dec(height(h), MAXPT);
    hh = gobjects(1,5);  ll = strings(1,5);

    % ---- linear speed ------------------------------------------------
    yyaxis(ax, "left")
    hold(ax, "on")
    if ~isempty(Wh{k})
        dw = dec(height(Wh{k}), MAXPT);
        hh(1) = plot(ax, Wh{k}.t_s(dw), Wh{k}.v_enc_mps(dw), "-", ...
                     "Color", Cd, "LineWidth", 1.4);
        ll(1) = "v encoder";
    end
    if ~all(isnan(h.v_meas_mps))
        hh(2) = plot(ax, h.t_s(d), h.v_meas_mps(d), "-", ...
                     "Color", [Cl 0.55], "LineWidth", 0.7);
        ll(2) = "v camera";
    end
    hh(3) = plot(ax, h.t_s(d), h.v_cmd_mps(d), "--", "Color", Cd, "LineWidth", 1.1);
    ll(3) = "v commanded";
    hold(ax, "off")
    ylabel(ax, "v [m/s]")
    ax.YAxis(1).Color = Cd;

    % ---- heading -----------------------------------------------------
    % Commanded heading = bearing to the lookahead point, theta + alpha
    yyaxis(ax, "right")
    hold(ax, "on")
    hh(4) = plot(ax, h.t_s(d), breakWraps(wrapDeg(h.theta_rad(d))), "-", ...
                 "Color", Cl, "LineWidth", 1.2);
    ll(4) = "\theta measured";
    hh(5) = plot(ax, h.t_s(d), breakWraps(wrapDeg(h.theta_rad(d) + h.alpha_rad(d))), "--", ...
                 "Color", Cl, "LineWidth", 1.0);
    ll(5) = "\theta commanded";
    hold(ax, "off")
    ylabel(ax, "\theta [deg]")
    ax.YAxis(2).Color = Cl;
    ylim(ax, [-190 190]), yticks(ax, -180:90:180)

    xlim(ax, [0 tMax])
    grid(ax, "on"), box(ax, "on")
    title(ax, VEH(k) + "  -  speed and heading, measured vs commanded", "Color", Cd)
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
function T = readIf(f)
T = [];
if isfile(f), T = readtable(f, "VariableNamingRule", "preserve"); end
end

function [t, x, y] = seenXY(h)
%SEENXY  Pose rows where the marker was seen, unique and in time order.
m = ~isnan(h.x_m) & ~isnan(h.y_m);
t = h.t_s(m); x = h.x_m(m); y = h.y_m(m);
[t, i] = unique(t);
x = x(i); y = y(i);
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
    if isscalar(m)
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
