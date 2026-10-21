%% Cooperative A* route planner - centralised joint-space search, CSV export
clear; clc; close all;

%% ================= USER-ADJUSTABLE PARAMETERS =================
% Movement area corners [xmin ymin; xmax ymax] (m)
area = [0 0; 2.4 1.6];

% Grid cell size (m) - must divide the area evenly
cellSize = 0.4;

% Vehicle start and goal points (m, on the area boundary) and well-clear radius (m)
v1.start = [2 0];       v1.goal = [0 1.2];    v1.radius = 0.4;
v2.start = [1.2 0];     v2.goal = [1.2 1.6];  v2.radius = 0.4;
v3.start = [2.4 1.2];   v3.goal = [0 0];      v3.radius = 0.4;
vehicles = {v1, v2, v3};

% Initial extra steps each vehicle may take over its solo route
detourBudget = 3;

% Largest detour budget tried before giving up
maxDetourBudget = 24;

% Detour budget increase per retry
budgetGrowth = 2;

% Maximum number of search nodes before the search is stopped
nodeLimit = 600000;

% Output waypoint file for the simulation
csvFilename = 'vehicle_paths.csv';

%% ================= DERIVED SETTINGS =================
tol = 1e-6;
xs = area(1,1):cellSize:area(2,1);
ys = area(1,2):cellSize:area(2,2);
nx = numel(xs);
ny = numel(ys);
dirs = [1 0; 0 1; -1 0; 0 -1]; % row index = heading: 1=East 2=North 3=West 4=South

numVehicles = numel(vehicles);

% The closed set packs the joint configuration into one double, which must
% stay an exact integer.
if (nx * ny * 8)^numVehicles > 2^53
    error(['Grid too large to pack %d vehicles into one state key ' ...
        '(%d x %d cells). Use a coarser cellSize or fewer vehicles.'], ...
        numVehicles, nx, ny);
end

%% ================= WELL-CLEAR REQUIREMENTS =================
for v = 1:numVehicles
    r = vehicles{v}.radius;
    if ~isscalar(r) || ~isfinite(r) || r < 0
        error('Vehicle %d has an invalid well-clear radius - it must be a finite scalar >= 0.', v);
    end
end
wellClearRadius = cellfun(@(veh) veh.radius, vehicles);   % 1 x N, m

% Required centre-to-centre separation of each pair, in metres and in cells
reqSepWorld = wellClearRadius' + wellClearRadius;
reqSepCells = reqSepWorld / cellSize;

% The same requirement as a vector, in separationHistory's pair order
reqPair = zeros(numVehicles*(numVehicles-1)/2, 1);
kPair = 0;
for a = 1:numVehicles-1
    for b = a+1:numVehicles
        kPair = kPair + 1;
        reqPair(kPair) = wellClearRadius(a) + wellClearRadius(b);
    end
end

for a = 1:numVehicles-1
    for b = a+1:numVehicles
        d0 = hypot(vehicles{a}.start(1)-vehicles{b}.start(1), ...
                   vehicles{a}.start(2)-vehicles{b}.start(2));
        if d0 < reqSepWorld(a,b) - 1e-9
            error(['Vehicles %d and %d start %.3f apart, closer than their combined ' ...
                   'well-clear radius = %.3f.'], a, b, d0, reqSepWorld(a,b));
        end
    end
end

%% ================= PER-VEHICLE HEURISTIC FIELDS AND SOLO ROUTES =================
startIJH = zeros(numVehicles, 3);
goalIJH  = zeros(numVehicles, 3);
hFields  = cell(numVehicles, 1);
soloCost = zeros(numVehicles, 1);
soloPaths = cell(numVehicles, 1);

timeField  = zeros(numVehicles, 1);
timeSolo   = zeros(numVehicles, 1);
timeJoint  = 0;
jointAttempts = 0;

% JIT warm-up so the timings below exclude one-off compile cost
warmField = distanceField(3, 3, dirs, 3, 2, 1);
walkDownhill(warmField, dirs, 3, 3, 1, 2, 1);
clear warmField;

for v = 1:numVehicles
    veh = vehicles{v};
    [si, sj] = xy2ij(veh.start(1), veh.start(2), xs, ys, tol);
    [gi, gj] = xy2ij(veh.goal(1),  veh.goal(2),  xs, ys, tol);
    sh = boundaryHeadingIn(veh.start(1), veh.start(2), area, tol);
    gh = boundaryHeadingOut(veh.goal(1), veh.goal(2), area, tol);

    startIJH(v,:) = [si sj sh];
    goalIJH(v,:)  = [gi gj gh];

    tId = tic;
    hFields{v} = distanceField(nx, ny, dirs, gi, gj, gh);
    timeField(v) = toc(tId);

    soloCost(v) = hFields{v}(si, sj, sh);
    if isinf(soloCost(v))
        error('Vehicle %d cannot reach its goal even with no other vehicles present.', v);
    end

    tId = tic;
    soloPaths{v} = walkDownhill(hFields{v}, dirs, nx, ny, si, sj, sh);
    timeSolo(v) = toc(tId);
end

%% ================= CENTRALIZED JOINT A* =================
P = struct();
P.N = numVehicles;
P.nx = nx;
P.ny = ny;
P.dirs = dirs;
P.hFields = hFields;
P.soloCost = soloCost;
P.startIJH = startIJH;
P.goalIJH = goalIJH;
P.nodeLimit = nodeLimit;
P.reqSepCells = reqSepCells;

fprintf('Centralized joint A* over %d vehicles (%d joint actions per timestep).\n', ...
    numVehicles, 3^numVehicles);
fprintf('Solo (unobstructed) costs:');
fprintf(' v%d = %d', [1:numVehicles; soloCost']);
fprintf(' steps.\n');
fprintf('Well-clear radii:');
fprintf(' v%d = %.2f', [1:numVehicles; wellClearRadius]);
fprintf('  ->  required pair separations:');
for a = 1:numVehicles-1
    for b = a+1:numVehicles
        fprintf(' v%d-v%d = %.2f', a, b, reqSepWorld(a,b));
    end
end
fprintf('\n');

B = detourBudget;
while true
    P.budget = B;
    P.maxT = max(soloCost) + B;

    tSearch = tic;
    [jointPaths, found, stats] = planJointAStar(P);
    elapsed = toc(tSearch);
    timeJoint = timeJoint + elapsed;
    jointAttempts = jointAttempts + 1;

    fprintf(['Detour budget B = %-2d : %7d nodes generated, %7d joint states expanded, ' ...
        '%8d branches tried (%.2f s)\n'], B, stats.generated, stats.expanded, stats.branches, elapsed);
    fprintf('                     pruned - %d out of bounds, %d over budget, %d collisions\n', ...
        stats.prunedBounds, stats.prunedBudget, stats.prunedConflict);

    if found
        break;
    end
    if stats.hitLimit
        error(['Search hit the %d node limit at budget B = %d without finding a plan. ' ...
            'Reduce detourBudget, coarsen the grid, or raise nodeLimit.'], nodeLimit, B);
    end
    if B >= maxDetourBudget
        error(['No collision-free joint plan exists even allowing every vehicle %d extra ' ...
            'steps. Check that the start/goal points are compatible and that the ' ...
            'well-clear radii are not too large for this grid.'], maxDetourBudget);
    end
    B = min(maxDetourBudget, B + budgetGrowth);
    fprintf('  -> no plan within that budget; retrying with B = %d.\n', B);
end

%% ================= RESULT SUMMARY =================
fprintf('\nJoint plan found (sum-of-costs optimal within a %d-step detour budget):\n', B);
totalSolo = 0;
totalJoint = 0;
for v = 1:numVehicles
    jointSteps = size(jointPaths{v}, 1) - 1;
    extraSteps = jointSteps - soloCost(v);
    totalSolo = totalSolo + soloCost(v);
    totalJoint = totalJoint + jointSteps;
    fprintf('  Vehicle %d: solo = %2d steps, joint = %2d steps', v, soloCost(v), jointSteps);
    if extraSteps > 0
        fprintf(' (+%d)\n', extraSteps);
    else
        fprintf(' (no detour needed)\n');
    end
end
fprintf('  Fleet total: %d steps vs %d unobstructed (+%d overall).\n\n', ...
    totalJoint, totalSolo, totalJoint - totalSolo);

%% ================= COMPUTATION TIME =================
timeSetup = sum(timeField) + sum(timeSolo);
timeTotal = timeSetup + timeJoint;

fprintf('COMPUTATION TIME (all %d vehicles)\n', numVehicles);
fprintf('  distance fields + solo routes   %9.4f s\n', timeSetup);
if jointAttempts > 1
    fprintf('  joint A* search (%d attempts)     %9.4f s\n', jointAttempts, timeJoint);
else
    fprintf('  joint A* search                 %9.4f s\n', timeJoint);
end
fprintf('  ------------------------------------------\n');
fprintf('  total to compute all routes     %9.4f s\n\n', timeTotal);

%% ================= CONVERT TO XY =================
xyJoint = cell(numVehicles, 1);
xySolo  = cell(numVehicles, 1);
for v = 1:numVehicles
    xyJoint{v} = [xs(jointPaths{v}(:,1))', ys(jointPaths{v}(:,2))'];
    xySolo{v}  = [xs(soloPaths{v}(:,1))',  ys(soloPaths{v}(:,2))'];
end

%% ================= EXPORT VEHICLE PATHS TO CSV =================
% One column pair per vehicle (v1_x, v1_y, ...), one row per timestep. A
% vehicle's columns are left empty once it has reached its goal.
nCsvRows = max(cellfun(@(p) size(p,1), xyJoint));

csvFid = fopen(csvFilename, 'w');
if csvFid == -1
    error(['Could not open "%s" for writing - if the file is currently open ' ...
        'in Excel or another program, close it and re-run.'], csvFilename);
end

for v = 1:numVehicles
    if v > 1
        fprintf(csvFid, ',');
    end
    fprintf(csvFid, 'v%d_x,v%d_y', v, v);
end
fprintf(csvFid, '\n');

for k = 1:nCsvRows
    for v = 1:numVehicles
        if v > 1
            fprintf(csvFid, ',');
        end
        if k <= size(xyJoint{v}, 1)
            fprintf(csvFid, '%.6f,%.6f', xyJoint{v}(k,1), xyJoint{v}(k,2));
        else
            fprintf(csvFid, ',');
        end
    end
    fprintf(csvFid, '\n');
end
fclose(csvFid);

fprintf('Wrote "%s": %d rows, %d column pairs, one waypoint per %.3f (= cellSize).\n', ...
    csvFilename, nCsvRows, numVehicles, cellSize);
for v = 1:numVehicles
    gaps = vecnorm(diff(xyJoint{v}), 2, 2);
    fprintf('  Vehicle %d: %3d waypoints, straight-line length %6.3f, spacing %.3f to %.3f\n', ...
        v, size(xyJoint{v},1), sum(gaps), min(gaps), max(gaps));
end
fprintf('\n');

%% ================= VERIFY THE SEPARATION CONSTRAINT =================
tracks = cell(numVehicles, 1);
for v = 1:numVehicles
    tracks{v} = struct('t', jointPaths{v}(:,4), 'x', xyJoint{v}(:,1), 'y', xyJoint{v}(:,2));
end
[~, pairDist, pairLabels] = separationHistory(tracks, 0.01);

slack = min(pairDist, [], 2, 'omitnan') - reqPair;
slack(isnan(slack)) = Inf;   % pair never shared any time
[worst, kW] = min(slack);
minSep = min(pairDist(kW,:), [], 'omitnan');

fprintf('Closest approach = %.3f (required >= %.3f for %s) -> %s\n', ...
    minSep, reqPair(kW), pairLabels{kW}, ternary(worst >= -1e-6, 'PASS', 'FAIL'));

%% ================= PLOT: SOLO VS JOINT ROUTES =================
COL = [0.00 0.45 0.74;      % v1 blue
       0.78 0.10 0.12;      % v2 red
       0.20 0.60 0.25];     % v3 green

dk = @(c) max(c*0.60, 0);            % dark shade  -> markers and labels
lt = @(c) c + (1-c)*0.58;            % light shade -> solo (reference) path

LBL_OFF = (area(2,2) - area(1,2))*0.03;

fig = figure('Name','Centralised joint A* planning','Color','w', ...
             'Position',[100 80 900 900]);
tl  = tiledlayout(fig, 1, 1, 'TileSpacing','compact', 'Padding','compact');
title(tl, 'Centralised joint A^* path planning', ...
      'FontWeight','bold', 'FontSize',14);

axMap = nexttile(tl);
hold(axMap, 'on')

rectangle(axMap, 'Position', [area(1,1) area(1,2) ...
                              area(2,1)-area(1,1) area(2,2)-area(1,2)], ...
          'EdgeColor', [0.25 0.25 0.25], 'LineWidth', 1.4, ...
          'HandleVisibility','off');

hJoint = gobjects(1, numVehicles);

for v = 1:numVehicles
    C = COL(v,:); Cd = dk(C); Cl = lt(C);
    veh = vehicles{v};

    plot(axMap, xySolo{v}(:,1), xySolo{v}(:,2), '--', ...
         'Color', Cl, 'LineWidth', 1.2, 'HandleVisibility','off');

    hJoint(v) = plot(axMap, xyJoint{v}(:,1), xyJoint{v}(:,2), '-', ...
                     'Color', C, 'LineWidth', 2.0, ...
                     'DisplayName', sprintf('Vehicle %d', v));

    plot(axMap, veh.start(1), veh.start(2), 'o', 'Color', Cd, ...
         'MarkerFaceColor','w', 'MarkerSize',7, 'LineWidth',1.3, ...
         'HandleVisibility','off');
    plot(axMap, veh.goal(1), veh.goal(2), 's', 'Color', Cd, ...
         'MarkerFaceColor', Cd, 'MarkerSize',8, 'HandleVisibility','off');

    if v == 3
        off = -LBL_OFF; va = 'top';
    else
        off =  LBL_OFF; va = 'bottom';
    end
    text(axMap, veh.start(1), veh.start(2) + off, 'Start', ...
         'Color', Cd, 'FontSize',8, 'FontWeight','bold', ...
         'HorizontalAlignment','center', 'VerticalAlignment', va);
    text(axMap, veh.goal(1), veh.goal(2) + off, 'End', ...
         'Color', Cd, 'FontSize',8, 'FontWeight','bold', ...
         'HorizontalAlignment','center', 'VerticalAlignment', va);
end

hold(axMap, 'off')
axis(axMap, 'equal'), grid(axMap, 'on'), box(axMap, 'off')
xlabel(axMap, 'x [m]'), ylabel(axMap, 'y [m]')
title(axMap, 'Solo plans (dashed) vs jointly planned (solid)')
legend(axMap, hJoint, 'Location','southoutside', 'Orientation','horizontal', ...
       'Box','off')
xlim(axMap, [area(1,1)-cellSize, area(2,1)+cellSize]);
ylim(axMap, [area(1,2)-cellSize, area(2,2)+cellSize]);

% Gridlines on the planning grid
xticks(axMap, (area(1,1)-cellSize):cellSize:(area(2,1)+cellSize));
yticks(axMap, (area(1,2)-cellSize):cellSize:(area(2,2)+cellSize));

%% ================= LOCAL FUNCTIONS =================
function [paths, found, stats] = planJointAStar(P)
% A* over the joint state of every vehicle, using operator decomposition
% (one vehicle moves per expansion). Per-vehicle status: 0 = driving,
% 1 = arrived this timestep, 2 = gone.
    N = P.N; nx = P.nx; ny = P.ny; dirs = P.dirs;
    reqSep = P.reqSepCells;
    goalI = P.goalIJH(:,1)'; goalJ = P.goalIJH(:,2)'; goalH = P.goalIJH(:,3)';

    stats = struct('generated', 0, 'expanded', 0, 'branches', 0, ...
        'prunedBounds', 0, 'prunedBudget', 0, 'prunedConflict', 0, 'hitLimit', false);
    paths = cell(N, 1);
    found = false;

    % --- node storage (preallocated, doubled on demand) ---
    cap = 8192;
    cfgI = zeros(cap, N); cfgJ = zeros(cap, N); cfgH = zeros(cap, N);
    prvI = zeros(cap, N); prvJ = zeros(cap, N);   % positions at start of timestep
    stat = zeros(cap, N); gPer = zeros(cap, N);
    tArr = zeros(cap, 1); mArr = zeros(cap, 1); parent = zeros(cap, 1);

    % --- root node ---
    nNodes = 1;
    cfgI(1,:) = P.startIJH(:,1)'; cfgJ(1,:) = P.startIJH(:,2)'; cfgH(1,:) = P.startIJH(:,3)';
    prvI(1,:) = cfgI(1,:); prvJ(1,:) = cfgJ(1,:);
    hRoot = 0;
    for k = 1:N
        hRoot = hRoot + P.hFields{k}(cfgI(1,k), cfgJ(1,k), cfgH(1,k));
    end

    heapIdx = zeros(cap, 1); heapKey = zeros(cap, 1); heapN = 0;
    [heapIdx, heapKey, heapN] = heapPush(heapIdx, heapKey, heapN, 1, hRoot);

    closed = containers.Map('KeyType', 'double', 'ValueType', 'logical');

    while heapN > 0
        [heapIdx, heapKey, heapN, cur] = heapPop(heapIdx, heapKey, heapN);

        baseI = cfgI(cur,:); baseJ = cfgJ(cur,:); baseH = cfgH(cur,:);
        baseS = stat(cur,:); baseG = gPer(cur,:);
        basePI = prvI(cur,:); basePJ = prvJ(cur,:);
        m = mArr(cur); t = tArr(cur);

        if m == 0
            % Only complete configurations are closed; time is not in the key
            key = encodeConfig(baseI, baseJ, baseH, baseS, nx, ny);
            if isKey(closed, key)
                continue;
            end
            closed(key) = true;
            stats.expanded = stats.expanded + 1;

            if all(baseS == 2)
                found = true;
                break;
            end
            if t >= P.maxT
                continue;
            end
        end

        if nNodes >= P.nodeLimit
            stats.hitLimit = true;
            return;
        end

        % ---- next vehicle to move ----
        a = 0;
        for k = (m+1):N
            if baseS(k) == 0
                a = k;
                break;
            end
        end

        for sel = 0:2
            stats.branches = stats.branches + 1;

            newI = baseI; newJ = baseJ; newH = baseH;
            newS = baseS; newG = baseG;

            switch sel
                case 0, nh = baseH(a);              % straight on
                case 1, nh = turnLeft(baseH(a));    % turn left, then move
                case 2, nh = turnRight(baseH(a));   % turn right, then move
            end
            ni = baseI(a) + dirs(nh,1);
            nj = baseJ(a) + dirs(nh,2);

            if ni < 1 || ni > nx || nj < 1 || nj > ny
                stats.prunedBounds = stats.prunedBounds + 1; continue;
            end
            hA = P.hFields{a}(ni, nj, nh);
            if isinf(hA)
                stats.prunedBounds = stats.prunedBounds + 1; continue;
            end
            % Detour budget: best finishing cost must fit in solo cost + slack
            if (baseG(a) + 1) + hA > P.soloCost(a) + P.budget
                stats.prunedBudget = stats.prunedBudget + 1; continue;
            end

            newI(a) = ni; newJ(a) = nj; newH(a) = nh; newG(a) = baseG(a) + 1;
            if ni == goalI(a) && nj == goalJ(a) && nh == goalH(a)
                newS(a) = 1;   % arrived; leaves the area at the rollover
            end

            % ---- check the mover against vehicles that already moved this timestep ----
            ok = true;
            for k = 1:m
                if k == a || baseS(k) == 2
                    continue;
                end
                sameCell = (newI(k) == newI(a)) && (newJ(k) == newJ(a));
                swapped  = (basePI(k) == newI(a)) && (basePJ(k) == newJ(a)) && ...
                           (newI(k) == baseI(a))  && (newJ(k) == baseJ(a));
                if sameCell || swapped || ...
                        checkPairSeparation(basePI(a), basePJ(a), newI(a), newJ(a), ...
                                            basePI(k), basePJ(k), newI(k), newJ(k), ...
                                            reqSep(a,k))
                    ok = false; break;
                end
            end
            if ~ok
                stats.prunedConflict = stats.prunedConflict + 1;
                continue;
            end

            % ---- roll the timestep over once every vehicle has moved ----
            newM = a;
            newPI = basePI; newPJ = basePJ;
            nextMover = 0;
            for k = (newM+1):N
                if newS(k) == 0
                    nextMover = k; break;
                end
            end
            newT = t;
            if nextMover == 0
                newT = t + 1;
                newM = 0;
                newPI = newI; newPJ = newJ;
                newS(newS == 1) = 2;   % arrived vehicles now leave the area
            end

            % ---- cost: fleet steps so far + sum of exact remaining distances ----
            gTot = sum(newG);
            hTot = 0;
            for k = 1:N
                hTot = hTot + P.hFields{k}(newI(k), newJ(k), newH(k));
            end

            % ---- store the node ----
            nNodes = nNodes + 1;
            if nNodes > cap
                cap = 2 * cap;
                cfgI(cap,N) = 0; cfgJ(cap,N) = 0; cfgH(cap,N) = 0;
                prvI(cap,N) = 0; prvJ(cap,N) = 0;
                stat(cap,N) = 0; gPer(cap,N) = 0;
                tArr(cap,1) = 0; mArr(cap,1) = 0; parent(cap,1) = 0;
            end
            cfgI(nNodes,:) = newI; cfgJ(nNodes,:) = newJ; cfgH(nNodes,:) = newH;
            prvI(nNodes,:) = newPI; prvJ(nNodes,:) = newPJ;
            stat(nNodes,:) = newS; gPer(nNodes,:) = newG;
            tArr(nNodes) = newT; mArr(nNodes) = newM; parent(nNodes) = cur;
            stats.generated = stats.generated + 1;

            % Tie-break towards deeper nodes
            [heapIdx, heapKey, heapN] = heapPush(heapIdx, heapKey, heapN, ...
                nNodes, (gTot + hTot) - 1e-6 * gTot);
        end
    end

    if ~found
        return;
    end

    % ---- reconstruct: keep only the complete (m = 0) configurations ----
    chain = zeros(tArr(cur) * (N+1) + 8, 1);
    nChain = 0;
    idx = cur;
    while idx ~= 0
        nChain = nChain + 1;
        chain(nChain) = idx;
        idx = parent(idx);
    end
    chain = flipud(chain(1:nChain));
    chain = chain(mArr(chain) == 0);

    for k = 1:N
        rows = zeros(numel(chain), 4);
        nRows = 0;
        for s = 1:numel(chain)
            nd = chain(s);
            nRows = nRows + 1;
            rows(nRows,:) = [cfgI(nd,k), cfgJ(nd,k), cfgH(nd,k), tArr(nd)];
            if stat(nd,k) == 2
                break;   % this is the timestep it reached its goal and left
            end
        end
        paths{k} = rows(1:nRows,:);
        if (nRows - 1) ~= gPer(cur,k)
            error('Internal error: vehicle %d path length %d disagrees with its step count %d.', ...
                k, nRows-1, gPer(cur,k));
        end
    end
end

function conflict = checkPairSeparation(ai0, aj0, ai1, aj1, bi0, bj0, bi1, bj1, reqSepCells)
% Closed-form closest approach of two vehicles over one timestep (grid units).
    r0x = ai0 - bi0;   r0y = aj0 - bj0;      % relative position at start of step
    r1x = ai1 - bi1;   r1y = aj1 - bj1;      % relative position at end of step
    drx = r1x - r0x;   dry = r1y - r0y;

    den = drx^2 + dry^2;
    if den < 1e-18
        s = 0;                                   % no relative motion
    else
        s = min(max(-(r0x*drx + r0y*dry) / den, 0), 1);
    end
    closest = hypot(r0x + s*drx, r0y + s*dry);

    conflict = closest < reqSepCells - 1e-9;
end

function [tGrid, pairDist, labels] = separationHistory(tracks, dt)
% Pairwise distances of time-parameterised tracks on a common time grid.
% A vehicle past its final time has left the area and is NaN.
    N = numel(tracks);
    durations = cellfun(@(s) s.t(end), tracks);
    tGrid = 0:dt:max(durations);

    PX = nan(N, numel(tGrid));
    PY = nan(N, numel(tGrid));
    for i = 1:N
        m = tGrid <= durations(i) + 1e-9;
        tq = min(tGrid(m), tracks{i}.t(end));
        PX(i,m) = interp1(tracks{i}.t, tracks{i}.x, tq);
        PY(i,m) = interp1(tracks{i}.t, tracks{i}.y, tq);
    end

    nPairs = N*(N-1)/2;
    pairDist = nan(nPairs, numel(tGrid));
    labels = cell(nPairs,1);
    k = 0;
    for a = 1:N-1
        for b = a+1:N
            k = k + 1;
            pairDist(k,:) = hypot(PX(a,:)-PX(b,:), PY(a,:)-PY(b,:));
            labels{k} = sprintf('Vehicle %d - Vehicle %d', a, b);
        end
    end
end

function out = ternary(cond, a, b)
    if cond; out = a; else; out = b; end
end

function hf = distanceField(nx, ny, dirs, gi, gj, gh)
% Exact steps-to-goal for every (cell, heading), by reverse BFS from the goal state.
    hf = inf(nx, ny, 4);
    hf(gi, gj, gh) = 0;

    queue = zeros(nx*ny*4, 3);
    queue(1,:) = [gi gj gh];
    head = 1; tail = 1;

    while head <= tail
        c = queue(head,:); head = head + 1;
        d = hf(c(1), c(2), c(3)) + 1;

        pi_ = c(1) - dirs(c(3),1);
        pj_ = c(2) - dirs(c(3),2);
        if pi_ < 1 || pi_ > nx || pj_ < 1 || pj_ > ny
            continue;
        end
        preHeadings = [c(3), turnLeft(c(3)), turnRight(c(3))];
        for k = 1:3
            ph = preHeadings(k);
            if hf(pi_, pj_, ph) > d
                hf(pi_, pj_, ph) = d;
                tail = tail + 1;
                queue(tail,:) = [pi_ pj_ ph];
            end
        end
    end
end

function path = walkDownhill(hf, dirs, nx, ny, si, sj, sh)
% Optimal solo route read off a distance field.
    d = hf(si, sj, sh);
    path = zeros(d+1, 4);
    cur = [si sj sh];
    for step = 0:(d-1)
        path(step+1,:) = [cur, step];
        candidates = [cur(3), turnLeft(cur(3)), turnRight(cur(3))];
        moved = false;
        for k = 1:3
            nh = candidates(k);
            ni = cur(1) + dirs(nh,1);
            nj = cur(2) + dirs(nh,2);
            if ni < 1 || ni > nx || nj < 1 || nj > ny
                continue;
            end
            if hf(ni, nj, nh) == hf(cur(1), cur(2), cur(3)) - 1
                cur = [ni nj nh];
                moved = true;
                break;
            end
        end
        if ~moved
            error('Internal error: distance field has no descending neighbour.');
        end
    end
    path(d+1,:) = [cur, d];
end

function key = encodeConfig(I, J, H, S, nx, ny)
% Packs a complete joint configuration into one number for the closed set.
    key = 0;
    for k = 1:numel(I)
        key = key * nx + (I(k) - 1);
        key = key * ny + (J(k) - 1);
        key = key * 4  + (H(k) - 1);
        key = key * 2  + double(S(k) == 2);
    end
end

function [hIdx, hKey, n] = heapPush(hIdx, hKey, n, idx, key)
% Binary min-heap push.
    n = n + 1;
    if n > numel(hIdx)
        hIdx(2*n) = 0; hKey(2*n) = 0;
    end
    hIdx(n) = idx; hKey(n) = key;
    c = n;
    while c > 1
        p = floor(c/2);
        if hKey(p) <= hKey(c)
            break;
        end
        tmpI = hIdx(p); tmpK = hKey(p);
        hIdx(p) = hIdx(c); hKey(p) = hKey(c);
        hIdx(c) = tmpI;   hKey(c) = tmpK;
        c = p;
    end
end

function [hIdx, hKey, n, top] = heapPop(hIdx, hKey, n)
% Binary min-heap pop of the lowest-f node.
    top = hIdx(1);
    hIdx(1) = hIdx(n); hKey(1) = hKey(n);
    n = n - 1;
    p = 1;
    while true
        l = 2*p; r = l + 1; s = p;
        if l <= n && hKey(l) < hKey(s), s = l; end
        if r <= n && hKey(r) < hKey(s), s = r; end
        if s == p
            break;
        end
        tmpI = hIdx(p); tmpK = hKey(p);
        hIdx(p) = hIdx(s); hKey(p) = hKey(s);
        hIdx(s) = tmpI;   hKey(s) = tmpK;
        p = s;
    end
end

function h2 = turnLeft(h)
    h2 = mod(h, 4) + 1; % headings ordered CCW: East->North->West->South
end

function h2 = turnRight(h)
    h2 = mod(h - 2, 4) + 1;
end

function [i, j] = xy2ij(x, y, xs, ys, tol)
    i = find(abs(xs - x) < tol, 1);
    j = find(abs(ys - y) < tol, 1);
    if isempty(i) || isempty(j)
        error('Point (%.4g, %.4g) does not align with the grid (check cellSize).', x, y);
    end
end

function h = boundaryHeadingIn(x, y, area, tol)
% Heading when entering the area at (x,y): points inward.
    xmin = area(1,1); ymin = area(1,2); xmax = area(2,1); ymax = area(2,2);
    if abs(x - xmin) < tol
        h = 1; % East
    elseif abs(x - xmax) < tol
        h = 3; % West
    elseif abs(y - ymin) < tol
        h = 2; % North
    elseif abs(y - ymax) < tol
        h = 4; % South
    else
        error('Point (%.4g, %.4g) is not on the boundary of the movement area.', x, y);
    end
end

function h = boundaryHeadingOut(x, y, area, tol)
% Heading when exiting the area at (x,y): points outward.
    hIn = boundaryHeadingIn(x, y, area, tol);
    h = mod(hIn + 1, 4) + 1;
end
