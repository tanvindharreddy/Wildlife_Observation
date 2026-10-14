%% LIVE WILDLIFE OBSERVATION ROBOT - MATLAB VERSION
% Reads live detections from:
%   C:\DCR project\data\vision_data.json
%
% Start vision_to_matlab_wildlife.py first and keep it running.
% Save this file as C:\DCR project\matlab\wildlife_matlab_live_auto_search.m
%
% IMPORTANT:
% The JSON contains 2-D camera detections, not real-world animal position.
% As in the former PyBullet demo, the first target's horizontal pixel position
% is mapped to a VIRTUAL target at an assumed distance. This is a simulation,
% not calibrated localization or physical robot control.

clc;
clearvars;

% Delete old figures directly. Unlike close all, delete bypasses stale
% CloseRequestFcn callbacks that may throw an error from a previous run.
oldFigures = findall(groot, 'Type', 'figure');
for figureIndex = 1:numel(oldFigures)
    if isgraphics(oldFigures(figureIndex))
        set(oldFigures(figureIndex), 'CloseRequestFcn', '');
        delete(oldFigures(figureIndex));
    end
end

%% -------------------------- CONFIGURATION -------------------------------
scriptFolder = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptFolder);
jsonFile = fullfile(projectRoot, 'data', 'vision_data.json');

WORLD_MIN = -40;
WORLD_MAX = 40;
GRID_SIZE = 100;
CELL_SIZE = (WORLD_MAX - WORLD_MIN) / GRID_SIZE;

START_XY = [0, -28];
START_YAW = pi/2;                    % robot initially faces +Y
SIM_TARGET_DISTANCE = 22;            % fallback if bbox data/calibration is unavailable
HORIZONTAL_FOV_DEG = 70;              % approximate; calibrate for your phone camera
MIN_ESTIMATED_RANGE = 2.0;             % metres; clamp for simulation stability
MAX_ESTIMATED_RANGE = 35.0;            % metres; clamp for simulation stability
OBSERVATION_DISTANCE = 5.0;
ROBOT_CLEARANCE = 1.35;
JSON_MAX_AGE = 3.0;                   % seconds
AUTO_UNLOCK_SECONDS = 3.5;           % reacquire automatically after target loss
SEARCH_YAW_RATE = deg2rad(22);        % simulated slow scan while no target is locked
SIM_DT = 1/30;
LIDAR_MAX_DISTANCE = 5.0;
LIDAR_RAYS = 9;
SAFETY_STOP_DISTANCE = 0.75;
BASE_SPEED = 5.0;                     % adjustable virtual speed (m/s); change this value to tune rover speed
MAX_YAW_RATE = 2.0;                   % rad/s
TURN_GAIN = 2.5;

% Habitat layout matches the previous wildlife PyBullet demo.
% Trees removed from both the visual scene and obstacle map.
% Rocks remain as obstacles for A* and simulated LiDAR.
ROCK_POSITIONS = [ ...
     -5 -19; 5 -16; 15 -1; -19 10; 22 12; ...
    -24  -8; 4   8; 25 -27; -13 22; 14 23];

% Conservative radii used by the grid planner; metres.
ROCK_MAP_RADIUS = 1.1;
obstacles = [ROCK_POSITIONS, repmat(ROCK_MAP_RADIUS, size(ROCK_POSITIONS,1), 1)];

%% -------------------------- CREATE FIGURE -------------------------------
fig = figure('Name', 'Live Wildlife Observation Robot - MATLAB', ...
    'NumberTitle', 'off', 'Color', [0.10 0.13 0.11], ...
    'KeyPressFcn', @keyPressCallback, 'CloseRequestFcn', @closeCallback);
setappdata(fig, 'quitRequested', false);
setappdata(fig, 'resetRequested', false);

ax = axes('Parent', fig, 'Position', [0.04 0.06 0.92 0.88]);
hold(ax, 'on');
axis(ax, [WORLD_MIN WORLD_MAX WORLD_MIN WORLD_MAX -0.2 7]);
axis(ax, 'equal');
view(ax, 48, 48);
set(ax, 'Color', [0.24 0.34 0.22], 'XColor', [0.92 0.92 0.92], ...
    'YColor', [0.92 0.92 0.92], 'ZColor', [0.92 0.92 0.92], ...
    'XGrid', 'on', 'YGrid', 'on', 'ZGrid', 'on', ...
    'GridColor', [0.60 0.66 0.57], 'GridAlpha', 0.18);
xlabel(ax, 'World X (m)', 'Color', 'w');
ylabel(ax, 'World Y (m)', 'Color', 'w');
zlabel(ax, 'Height (m)', 'Color', 'w');

% Slightly uneven green terrain.
terrainN = 55;
[terrainX, terrainY] = meshgrid(linspace(WORLD_MIN, WORLD_MAX, terrainN));
terrainZ = 0.035*sin(terrainX/4.8).*cos(terrainY/5.5) + ...
           0.025*cos(terrainX/7.0 + terrainY/8.0);
surf(ax, terrainX, terrainY, terrainZ, ...
    'FaceColor', [0.28 0.46 0.24], 'EdgeColor', 'none', 'FaceAlpha', 1);

% Ground habitat patches (visual only).
patchCenters = [-7 7; 10 4; -18 -1; 19 -11; 0 18; -25 14];
for k = 1:size(patchCenters,1)
    [bx, by, bz] = sphere(12);
    surf(ax, patchCenters(k,1)+2.2*bx, patchCenters(k,2)+1.6*by, ...
        0.04+0.025*bz, 'FaceColor', [0.34 0.40 0.20], ...
        'EdgeColor', 'none', 'FaceAlpha', 0.65);
end

% Trees intentionally omitted from this version.

% Draw rocks as low ellipsoids.
for k = 1:size(ROCK_POSITIONS,1)
    rx = ROCK_POSITIONS(k,1); ry = ROCK_POSITIONS(k,2);
    [sx, sy, sz] = sphere(10);
    surf(ax, rx+0.85*sx, ry+0.72*sy, 0.55+0.55*sz, ...
        'FaceColor', [0.39 0.40 0.37], 'EdgeColor', 'none');
end

% World boundary, to make the operating area visible.
plot3(ax, [WORLD_MIN WORLD_MAX WORLD_MAX WORLD_MIN WORLD_MIN], ...
    [WORLD_MIN WORLD_MIN WORLD_MAX WORLD_MAX WORLD_MIN], ...
    0.08*ones(1,5), '--', 'Color', [0.88 0.83 0.45], 'LineWidth', 1.4);

% Static occupancy grid used by A*.
occupancy = buildOccupancyMap(obstacles, WORLD_MIN, CELL_SIZE, GRID_SIZE, ROBOT_CLEARANCE);

% Initial robot state.
robotXY = START_XY;
robotYaw = START_YAW;
route = zeros(0,2);
routeIndex = 1;
observationGoal = [NaN NaN];
virtualTarget = [NaN NaN];
lockedTrackId = NaN;
lockedClass = '';
missingSince = NaN;
missionState = 'SEARCHING FOR WILDLIFE';

% Visual handles for the robot.
bodyFaces = [1 2 3 4; 5 8 7 6; 1 5 6 2; 2 6 7 3; 3 7 8 4; 4 8 5 1];
bodyV = orientedBoxVertices(robotXY, robotYaw, 2.0, 1.25, 0.48, 0.95, 0, 0);
hRobotBody = patch(ax, 'Vertices', bodyV, 'Faces', bodyFaces, ...
    'FaceColor', [0.12 0.25 0.32], 'EdgeColor', [0.05 0.07 0.08], 'LineWidth', 1.2);
% Top equipment panel.
topV = orientedBoxVertices(robotXY, robotYaw, 0.95, 0.78, 0.96, 1.18, -0.05, 0);
hRobotTop = patch(ax, 'Vertices', topV, 'Faces', bodyFaces, ...
    'FaceColor', [0.19 0.36 0.42], 'EdgeColor', [0.04 0.06 0.07]);

wheelOffsets = [0.60 -0.75; 0.60 0.75; -0.60 -0.75; -0.60 0.75];
hWheels = gobjects(4,1);
for k = 1:4
    wv = orientedBoxVertices(robotXY, robotYaw, 0.52, 0.26, 0.15, 0.55, ...
        wheelOffsets(k,1), wheelOffsets(k,2));
    hWheels(k) = patch(ax, 'Vertices', wv, 'Faces', bodyFaces, ...
        'FaceColor', [0.035 0.04 0.04], 'EdgeColor', [0.01 0.01 0.01]);
end
% Front camera/sensor direction marker.
frontLine = plot3(ax, [robotXY(1) robotXY(1)+1.6*cos(robotYaw)], ...
    [robotXY(2) robotXY(2)+1.6*sin(robotYaw)], [1.0 1.0], ...
    '-', 'Color', [0.2 0.9 1.0], 'LineWidth', 3);

hRoute = plot3(ax, NaN, NaN, NaN, '-', 'Color', [0.1 0.55 1.0], 'LineWidth', 2.7);
hGoal = plot3(ax, NaN, NaN, NaN, 'o', 'MarkerSize', 12, ...
    'MarkerFaceColor', [0.1 0.95 0.25], 'MarkerEdgeColor', 'w', 'LineWidth', 1.5);
hTargetLink = plot3(ax, NaN, NaN, NaN, '--', 'Color', [1.0 0.78 0.15], 'LineWidth', 1.3);

% Simulated front LiDAR fan. Rays are visual guides, not actual hardware data.
lidarHandles = gobjects(LIDAR_RAYS,1);
for k = 1:LIDAR_RAYS
    lidarHandles(k) = plot3(ax, [0 0], [0 0], [0.78 0.78], ...
        ':', 'Color', [0.75 0.80 0.72], 'LineWidth', 0.9);
end

animalHandles = {};

% Logs for end-of-run plots (like the original robot demo).
startClock = tic;
timeLog = [];
distanceLog = [];
speedLog = [];
modeLog = {};
robotXLog = [];
robotYLog = [];
obstacleEncounter = [];

fprintf('\n============================================================\n');
fprintf('LIVE WILDLIFE OBSERVATION ROBOT - MATLAB\n');
fprintf('JSON input: %s\n', jsonFile);
fprintf('Keep vision_to_matlab_wildlife.py running in another window.\n');
fprintf('Keys: R = manual reset, Q = quit. Automatic search/reacquisition is enabled.\n');
fprintf('Target world distance is assumed for simulation, not measured.\n');
fprintf('============================================================\n\n');

%% ------------------------------ MAIN LOOP -------------------------------
lastTick = tic;
while isgraphics(fig) && ~getappdata(fig, 'quitRequested')
    loopTimer = tic;

    % Reset target selection if R was pressed.
    if getappdata(fig, 'resetRequested')
        setappdata(fig, 'resetRequested', false);
        if ~isempty(animalHandles)
            for animalHandleIndex = 1:numel(animalHandles)
                if isgraphics(animalHandles{animalHandleIndex})
                    delete(animalHandles{animalHandleIndex});
                end
            end
        end
        animalHandles = {};
        lockedTrackId = NaN;
        lockedClass = '';
        missingSince = NaN;
        virtualTarget = [NaN NaN];
        observationGoal = [NaN NaN];
        route = zeros(0,2);
        routeIndex = 1;
        set(hRoute, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
        set(hGoal, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
        set(hTargetLink, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
        missionState = 'SEARCHING FOR WILDLIFE';
        fprintf('Local target cleared. Automatic searching remains active.\n');
    end

    det = readJsonDetection(jsonFile, JSON_MAX_AGE);
    targetMatchesLock = false;
    elapsedNow = toc(startClock);

    if ~isempty(det)
        % Any fresh detector lock is accepted automatically. If the Python
        % tracker has selected a replacement ID after losing the old target,
        % MATLAB releases the old virtual animal and follows the new one.
        hadMissingInterval = ~isnan(missingSince);
        missingSince = NaN;
        newTarget = isnan(lockedTrackId) || hadMissingInterval || ...
            det.trackId ~= lockedTrackId || ~strcmpi(det.className, lockedClass);

        if newTarget
            if ~isnan(lockedTrackId)
                fprintf('Target %s (ID %g) changed/lost; automatically reacquiring %s (ID %g).\n', ...
                    lockedClass, lockedTrackId, det.className, det.trackId);
            else
                fprintf('Searching found wildlife: %s, track ID %g.\n', det.className, det.trackId);
            end

            if ~isempty(animalHandles)
                for animalHandleIndex = 1:numel(animalHandles)
                    if isgraphics(animalHandles{animalHandleIndex})
                        delete(animalHandles{animalHandleIndex});
                    end
                end
            end
            animalHandles = {};
            set(hRoute, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
            set(hGoal, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
            set(hTargetLink, 'XData', NaN, 'YData', NaN, 'ZData', NaN);

            lockedTrackId = det.trackId;
            lockedClass = det.className;

            % Estimate range from bbox height, then map image horizontal position
            % to a virtual bearing. Phone movement/tilt is not tracked here.
            estimatedRange = estimateAnimalRange(det.className, det.bboxHeight, ...
                det.frameWidth, HORIZONTAL_FOV_DEG, MIN_ESTIMATED_RANGE, ...
                MAX_ESTIMATED_RANGE, SIM_TARGET_DISTANCE);
            frameMid = det.frameWidth / 2;
            normX = max(-1, min(1, (det.pixelX - frameMid) / frameMid));
            bearing = normX * deg2rad(HORIZONTAL_FOV_DEG/2);
            worldBearing = robotYaw + bearing;
            virtualTarget = robotXY + estimatedRange * [cos(worldBearing), sin(worldBearing)];
            virtualTarget = max(WORLD_MIN+2, min(WORLD_MAX-2, virtualTarget));

            [observationGoal, route] = findObservationRoute(virtualTarget, robotXY, ...
                occupancy, WORLD_MIN, WORLD_MAX, CELL_SIZE, GRID_SIZE, OBSERVATION_DISTANCE);
            routeIndex = min(2, size(route,1));

            % Draw the currently locked virtual animal.
            [animalHandles, ~] = drawVirtualAnimal(ax, virtualTarget, lockedClass);
            set(hTargetLink, 'XData', [robotXY(1) virtualTarget(1)], ...
                'YData', [robotXY(2) virtualTarget(2)], 'ZData', [1.0 0.2]);

            if all(isfinite(observationGoal)) && ~isempty(route)
                routeXY = gridPathToWorld(route, WORLD_MIN, CELL_SIZE);
                set(hRoute, 'XData', routeXY(:,1), 'YData', routeXY(:,2), ...
                    'ZData', 0.16*ones(size(routeXY,1),1));
                set(hGoal, 'XData', observationGoal(1), 'YData', observationGoal(2), 'ZData', 0.35);
                missionState = sprintf('NAVIGATING TO OBSERVE %s (ID %g)', upper(lockedClass), lockedTrackId);
                fprintf('Virtual target: (%.1f, %.1f) m; observation goal: (%.1f, %.1f) m\n', ...
                    virtualTarget(1), virtualTarget(2), observationGoal(1), observationGoal(2));
                fprintf('A* route cells: %d. Estimated camera-to-target range: %.1f m (rough estimate).\n', ...
                    size(route,1), estimatedRange);
            else
                missionState = 'NO SAFE A* ROUTE - ROBOT STOPPED';
                fprintf('No safe observation route found. The system will keep searching after target loss.\n');
            end
            targetMatchesLock = true;
        else
            targetMatchesLock = true;
        end
    else
        if ~isnan(lockedTrackId)
            if isnan(missingSince)
                missingSince = elapsedNow;
            end
            missingDuration = elapsedNow - missingSince;
            missionState = sprintf('TARGET LOST - AUTO SEARCH IN %.1f s', ...
                max(0, AUTO_UNLOCK_SECONDS-missingDuration));

            % Do not require R to resume searching. Release the lock after a
            % short grace period; Python is configured to auto-select a new
            % visible animal after its own LOST_TIMEOUT.
            if missingDuration >= AUTO_UNLOCK_SECONDS
                fprintf('Target %s (ID %g) unavailable. Returning to SEARCHING automatically.\n', ...
                    lockedClass, lockedTrackId);
                if ~isempty(animalHandles)
                    for animalHandleIndex = 1:numel(animalHandles)
                        if isgraphics(animalHandles{animalHandleIndex})
                            delete(animalHandles{animalHandleIndex});
                        end
                    end
                end
                animalHandles = {};
                lockedTrackId = NaN;
                lockedClass = '';
                virtualTarget = [NaN NaN];
                observationGoal = [NaN NaN];
                route = zeros(0,2);
                routeIndex = 1;
                set(hRoute, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
                set(hGoal, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
                set(hTargetLink, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
                missingSince = NaN;
                missionState = 'SEARCHING FOR WILDLIFE';
            end
        else
            missionState = 'SEARCHING FOR WILDLIFE';
        end
    end

    % No target lock: stay in searching state and rotate slowly in place.
    % Target lock: navigate only when fresh JSON data confirms that lock.
    robotSpeed = 0;
    currentMode = 'SEARCHING';
    dt = toc(lastTick);
    dt = min(max(dt, 0.005), 0.08);

    if isnan(lockedTrackId)
        robotYaw = wrapAngle(robotYaw + SEARCH_YAW_RATE*dt);
        missionState = 'SEARCHING FOR WILDLIFE';
    elseif targetMatchesLock && ~isempty(route) && all(isfinite(observationGoal))
        [robotXY, robotYaw, routeIndex, currentMode, robotSpeed, frontDistance] = ...
            advanceRobot(robotXY, robotYaw, route, routeIndex, observationGoal, ...
            obstacles, dt, BASE_SPEED, TURN_GAIN, MAX_YAW_RATE, ...
            LIDAR_RAYS, LIDAR_MAX_DISTANCE, SAFETY_STOP_DISTANCE, ...
            WORLD_MIN, CELL_SIZE, GRID_SIZE);
        if strcmp(currentMode, 'OBSERVING')
            missionState = sprintf('OBSERVING %s (ID %g)', upper(lockedClass), lockedTrackId);
        elseif strcmp(currentMode, 'SAFETY STOP')
            missionState = 'LIDAR SAFETY STOP - CHECK ROUTE';
            obstacleEncounter(end+1,:) = [robotXY, toc(startClock)]; %#ok<SAGROW>
        elseif ~strcmp(currentMode, 'STOPPED')
            missionState = sprintf('NAVIGATING TO OBSERVE %s (ID %g)', upper(lockedClass), lockedTrackId);
        end
        if frontDistance < 2.0
            obstacleEncounter(end+1,:) = [robotXY, toc(startClock)]; %#ok<SAGROW>
        end
    else
        currentMode = 'STOPPED';
    end
    lastTick = tic;

    % Update robot geometry in its new pose.
    set(hRobotBody, 'Vertices', orientedBoxVertices(robotXY, robotYaw, 2.0, 1.25, 0.48, 0.95, 0, 0));
    set(hRobotTop, 'Vertices', orientedBoxVertices(robotXY, robotYaw, 0.95, 0.78, 0.96, 1.18, -0.05, 0));
    for k = 1:4
        set(hWheels(k), 'Vertices', orientedBoxVertices(robotXY, robotYaw, 0.52, 0.26, 0.15, 0.55, ...
            wheelOffsets(k,1), wheelOffsets(k,2)));
    end
    set(frontLine, 'XData', [robotXY(1) robotXY(1)+1.6*cos(robotYaw)], ...
        'YData', [robotXY(2) robotXY(2)+1.6*sin(robotYaw)], 'ZData', [1.0 1.0]);

    % Update visible LiDAR rays (nearest circular obstacle hit per ray).
    frontDistanceVisual = updateLidarGraphics(lidarHandles, robotXY, robotYaw, ...
        obstacles, LIDAR_RAYS, LIDAR_MAX_DISTANCE);
    if ~isempty(route) && all(isfinite(observationGoal))
        distanceToGoal = hypot(observationGoal(1)-robotXY(1), observationGoal(2)-robotXY(2));
    else
        distanceToGoal = NaN;
    end

    elapsed = toc(startClock);
    timeLog(end+1,1) = elapsed; %#ok<SAGROW>
    distanceLog(end+1,1) = distanceToGoal; %#ok<SAGROW>
    speedLog(end+1,1) = robotSpeed; %#ok<SAGROW>
    modeLog{end+1,1} = currentMode; %#ok<SAGROW>
    robotXLog(end+1,1) = robotXY(1); %#ok<SAGROW>
    robotYLog(end+1,1) = robotXY(2); %#ok<SAGROW>

    title(ax, sprintf('LIVE WILDLIFE OBSERVATION ROBOT | %s | Speed %.1f m/s\nJSON: vision_data.json  |  R: manual reset  |  Q: quit', ...
        missionState, BASE_SPEED), 'Color', [1 1 1], 'FontSize', 12, 'Interpreter', 'none');
    drawnow limitrate;
    pause(max(0, SIM_DT - toc(loopTimer)));
end

% Close the main window and show results, like the original simulation's plots.
if isgraphics(fig)
    delete(fig);
end
plotSimulationResults(timeLog, distanceLog, speedLog, modeLog, robotXLog, robotYLog, ...
    START_XY, obstacleEncounter, observationGoal);
fprintf('MATLAB wildlife simulation closed.\n');

%% ============================ LOCAL FUNCTIONS ============================
function det = readJsonDetection(jsonFile, maxAge)
% Return a detection only when it is marked found and timestamp is fresh.
    det = [];
    try
        if ~isfile(jsonFile)
            return;
        end
        data = jsondecode(fileread(jsonFile));
        if ~isstruct(data) || ~isfield(data, 'target') || ~isfield(data, 'timestamp') || ~isfield(data, 'frame')
            return;
        end
        target = data.target;
        if ~isstruct(target) || ~isfield(target, 'found') || ~logical(target.found)
            return;
        end
        if isfield(target, 'target_lost') && ~isempty(target.target_lost) && logical(target.target_lost)
            return;
        end
        ts = double(data.timestamp);
        % Convert MATLAB's current UTC time to Unix seconds, matching Python time.time().
        nowUnix = posixtime(datetime('now', 'TimeZone', 'UTC'));
        if ~isfinite(ts) || ts <= 0 || (nowUnix-ts) > maxAge
            return;
        end
        if ~isfield(data.frame, 'width') || isempty(data.frame.width) || double(data.frame.width) <= 0
            return;
        end
        if ~isfield(target, 'pixel_x') || isempty(target.pixel_x) || ...
                ~isfield(target, 'height') || isempty(target.height) || ...
                ~isfield(target, 'class') || isempty(target.class) || ...
                ~isfield(target, 'target_id') || isempty(target.target_id)
            return;
        end
        trackID = double(target.target_id);
        pixelX = double(target.pixel_x);
        frameWidth = double(data.frame.width);
        bboxHeight = double(target.height);
        if ~isfinite(trackID) || ~isfinite(pixelX) || ~isfinite(frameWidth) || ...
                ~isfinite(bboxHeight) || frameWidth <= 0 || bboxHeight <= 0
            return;
        end
        det = struct('trackId', trackID, 'className', char(target.class), ...
            'pixelX', pixelX, 'frameWidth', frameWidth, ...
            'bboxHeight', bboxHeight, 'timestamp', ts);
    catch
        % The detector uses an atomic temp-file rename. If a read overlaps a
        % write or the file is absent, wait for the next loop iteration.
        det = [];
    end
end

function rangeM = estimateAnimalRange(className, bboxHeightPx, frameWidthPx, fovDeg, minRange, maxRange, fallbackRange)
%ESTIMATEANIMALRANGE Rough monocular range estimate from bbox height.
% range ~= focalLengthPixels * assumedAnimalHeightMetres / bboxHeightPixels.
% Species height is approximate; posture, orientation, detector-box tightness,
% and phone movement can cause substantial error. This is not ground truth.

    switch lower(strtrim(className))
        case 'buffalo'
            assumedHeightM = 1.5;
        case 'elephant'
            assumedHeightM = 2.8;
        case 'rhino'
            assumedHeightM = 1.7;
        case 'zebra'
            assumedHeightM = 1.4;
        otherwise
            assumedHeightM = NaN;
    end

    if ~isfinite(bboxHeightPx) || bboxHeightPx <= 1 || ...
            ~isfinite(frameWidthPx) || frameWidthPx <= 0 || ...
            ~isfinite(fovDeg) || fovDeg <= 1 || fovDeg >= 179 || ...
            ~isfinite(assumedHeightM)
        rangeM = fallbackRange;
        return;
    end

    % Focal length in pixels derived from horizontal FOV; assumes square pixels.
    focalLengthPx = frameWidthPx / (2 * tand(fovDeg/2));
    rawRangeM = focalLengthPx * assumedHeightM / bboxHeightPx;

    if ~isfinite(rawRangeM) || rawRangeM <= 0
        rangeM = fallbackRange;
    else
        rangeM = max(minRange, min(maxRange, rawRangeM));
    end
end

function occupancy = buildOccupancyMap(obstacles, worldMin, cellSize, gridSize, clearance)
% Occupancy array is indexed as occupancy(gridY, gridX).
    occupancy = false(gridSize, gridSize);
    for k = 1:size(obstacles,1)
        ox = obstacles(k,1); oy = obstacles(k,2);
        inflatedRadius = clearance + obstacles(k,3);
        gx0 = floor((ox-worldMin)/cellSize) + 1;
        gy0 = floor((oy-worldMin)/cellSize) + 1;
        n = ceil(inflatedRadius/cellSize) + 1;
        for gy = max(1,gy0-n):min(gridSize,gy0+n)
            wy = worldMin + (gy-0.5)*cellSize;
            for gx = max(1,gx0-n):min(gridSize,gx0+n)
                wx = worldMin + (gx-0.5)*cellSize;
                if hypot(wx-ox, wy-oy) <= inflatedRadius
                    occupancy(gy,gx) = true;
                end
            end
        end
    end
end

function [bestGoal, bestPath] = findObservationRoute(targetXY, robotXY, occupancy, ...
    worldMin, worldMax, cellSize, gridSize, observationDistance)
    bestGoal = [NaN NaN];
    bestPath = zeros(0,2);
    startCell = worldToGrid(robotXY(1), robotXY(2), worldMin, cellSize, gridSize);
    bestScore = inf;
    numCandidates = 24;
    for k = 1:numCandidates
        angle = 2*pi*(k-1)/numCandidates;
        candidate = targetXY + observationDistance*[cos(angle), sin(angle)];
        if any(candidate <= worldMin+1) || any(candidate >= worldMax-1)
            continue;
        end
        goalCell = worldToGrid(candidate(1), candidate(2), worldMin, cellSize, gridSize);
        if occupancy(goalCell(2), goalCell(1))
            continue;
        end
        candidatePath = astarGrid(occupancy, startCell, goalCell);
        if isempty(candidatePath)
            continue;
        end
        if size(candidatePath,1) > 1
            pathCost = sum(sqrt(sum(diff(candidatePath,1,1).^2,2)));
        else
            pathCost = 0;
        end
        score = pathCost + 0.015*hypot(candidate(1)-robotXY(1), candidate(2)-robotXY(2));
        if score < bestScore
            bestScore = score;
            bestGoal = candidate;
            bestPath = candidatePath;
        end
    end
end

function cellXY = worldToGrid(x, y, worldMin, cellSize, gridSize)
    gx = floor((x-worldMin)/cellSize) + 1;
    gy = floor((y-worldMin)/cellSize) + 1;
    gx = max(1, min(gridSize, gx));
    gy = max(1, min(gridSize, gy));
    cellXY = [gx gy];
end

function worldXY = gridPathToWorld(path, worldMin, cellSize)
    worldXY = [worldMin+(path(:,1)-0.5)*cellSize, ...
               worldMin+(path(:,2)-0.5)*cellSize];
end

function path = astarGrid(occupancy, startCell, goalCell)
% Eight-connected A* with octile heuristic and diagonal corner-cut prevention.
    [gridSize, ~] = size(occupancy);
    sx = startCell(1); sy = startCell(2);
    gx = goalCell(1); gy = goalCell(2);
    path = zeros(0,2);
    if sx<1 || sy<1 || gx<1 || gy<1 || sx>gridSize || sy>gridSize || gx>gridSize || gy>gridSize
        return;
    end
    if occupancy(sy,sx) || occupancy(gy,gx)
        return;
    end
    if sx == gx && sy == gy
        path = [sx sy];
        return;
    end

    gScore = inf(gridSize, gridSize);
    gScore(sy,sx) = 0;
    cameX = zeros(gridSize,gridSize);
    cameY = zeros(gridSize,gridSize);
    closed = false(gridSize,gridSize);
    heap = zeros(20000,4); % row: [fScore, x, y, gScore]
    heapN = 0;
    [heap,heapN] = heapPush(heap,heapN,[octileHeuristic(sx,sy,gx,gy),sx,sy,0]);
    neighbors = [1 0 1; -1 0 1; 0 1 1; 0 -1 1; ...
                 1 1 sqrt(2); 1 -1 sqrt(2); -1 1 sqrt(2); -1 -1 sqrt(2)];

    while heapN > 0
        [heap,heapN,node] = heapPop(heap,heapN);
        x = node(2); y = node(3); poppedG = node(4);
        if closed(y,x) || poppedG > gScore(y,x) + 1e-9
            continue;
        end
        if x == gx && y == gy
            n = 1;
            reversePath = zeros(gridSize*gridSize,2);
            reversePath(n,:) = [x y];
            while ~(x == sx && y == sy)
                px = cameX(y,x); py = cameY(y,x);
                if px == 0 || py == 0
                    path = zeros(0,2);
                    return;
                end
                x = px; y = py;
                n = n + 1;
                reversePath(n,:) = [x y];
            end
            path = flipud(reversePath(1:n,:));
            return;
        end
        closed(y,x) = true;
        for j = 1:size(neighbors,1)
            dx = neighbors(j,1); dy = neighbors(j,2); step = neighbors(j,3);
            nx = x+dx; ny = y+dy;
            if nx<1 || ny<1 || nx>gridSize || ny>gridSize || occupancy(ny,nx) || closed(ny,nx)
                continue;
            end
            if dx~=0 && dy~=0
                if occupancy(y,nx) || occupancy(ny,x)
                    continue;
                end
            end
            tentative = gScore(y,x) + step;
            if tentative < gScore(ny,nx)
                gScore(ny,nx) = tentative;
                cameX(ny,nx) = x;
                cameY(ny,nx) = y;
                f = tentative + octileHeuristic(nx,ny,gx,gy);
                [heap,heapN] = heapPush(heap,heapN,[f,nx,ny,tentative]);
            end
        end
    end
end

function h = octileHeuristic(x1,y1,x2,y2)
    dx = abs(x1-x2); dy = abs(y1-y2);
    h = max(dx,dy) + (sqrt(2)-1)*min(dx,dy);
end

function [heap,n] = heapPush(heap,n,item)
    n = n+1;
    if n > size(heap,1)
        heap = [heap; zeros(size(heap,1),4)]; %#ok<AGROW>
    end
    heap(n,:) = item;
    i = n;
    while i > 1
        parent = floor(i/2);
        if heap(parent,1) <= heap(i,1)
            break;
        end
        temp = heap(parent,:); heap(parent,:) = heap(i,:); heap(i,:) = temp;
        i = parent;
    end
end

function [heap,n,item] = heapPop(heap,n)
    item = heap(1,:);
    if n == 1
        n = 0;
        return;
    end
    heap(1,:) = heap(n,:);
    n = n-1;
    i = 1;
    while true
        left = 2*i; right = left+1; smallest = i;
        if left <= n && heap(left,1) < heap(smallest,1)
            smallest = left;
        end
        if right <= n && heap(right,1) < heap(smallest,1)
            smallest = right;
        end
        if smallest == i
            break;
        end
        temp = heap(i,:); heap(i,:) = heap(smallest,:); heap(smallest,:) = temp;
        i = smallest;
    end
end

function V = orientedBoxVertices(center, yaw, lengthX, widthY, zBottom, zTop, offsetX, offsetY)
    localXY = [offsetX-lengthX/2 offsetY-widthY/2; ...
               offsetX+lengthX/2 offsetY-widthY/2; ...
               offsetX+lengthX/2 offsetY+widthY/2; ...
               offsetX-lengthX/2 offsetY+widthY/2];
    R = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];
    worldXY = (R*localXY')' + center;
    V = [worldXY repmat(zBottom,4,1); worldXY repmat(zTop,4,1)];
end

function [handles, color] = drawVirtualAnimal(ax, xy, species)
    switch lower(species)
        case 'buffalo'
            color = [0.30 0.22 0.14];
        case 'elephant'
            color = [0.48 0.50 0.53];
        case 'rhino'
            color = [0.35 0.39 0.36];
        case 'zebra'
            color = [0.88 0.87 0.80];
        otherwise
            color = [0.55 0.45 0.30];
    end
    [sx,sy,sz] = sphere(14);
    hBody = surf(ax, xy(1)+1.25*sx, xy(2)+0.72*sy, 1.0+0.63*sz, ...
        'FaceColor', color, 'EdgeColor', 'none');
    hHead = surf(ax, xy(1)+0.95+0.36*sx, xy(2)+0.02+0.34*sy, 1.12+0.34*sz, ...
        'FaceColor', color, 'EdgeColor', 'none');
    hLabel = text(ax, xy(1), xy(2), 2.5, sprintf('VIRTUAL TARGET: %s', upper(species)), ...
        'Color', [1 1 1], 'FontSize', 10, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center', 'Interpreter', 'none');
    handles = {hBody; hHead; hLabel};
end

function [newXY, newYaw, routeIndex, mode, speed, frontDistance] = advanceRobot(robotXY, robotYaw, route, routeIndex, ...
    goalXY, obstacles, dt, baseSpeed, turnGain, maxYawRate, numRays, maxLidar, safetyStopDistance, ...
    worldMin, cellSize, gridSize)
    newXY = robotXY;
    newYaw = robotYaw;
    speed = 0;
    mode = 'STOPPED';
    frontDistance = maxLidar;

    if isempty(route) || any(~isfinite(goalXY))
        return;
    end
    goalDist = hypot(goalXY(1)-robotXY(1), goalXY(2)-robotXY(2));
    if goalDist < 1.25
        mode = 'OBSERVING';
        return;
    end

    % Advance over waypoints that the robot has already reached.
    routeXY = gridPathToWorld(route, worldMin, cellSize);
    while routeIndex < size(routeXY,1) && hypot(routeXY(routeIndex,1)-robotXY(1), routeXY(routeIndex,2)-robotXY(2)) < 0.48
        routeIndex = routeIndex + 1;
    end
    routeIndex = max(1, min(routeIndex, size(routeXY,1)));
    waypoint = routeXY(routeIndex,:);
    if routeIndex == 1 && size(routeXY,1) > 1
        waypoint = routeXY(2,:);
    end
    targetAngle = atan2(waypoint(2)-robotXY(2), waypoint(1)-robotXY(1));

    frontDistance = frontLidarDistance(robotXY, robotYaw, obstacles, numRays, maxLidar);
    if frontDistance < safetyStopDistance
        mode = 'SAFETY STOP';
        return;
    end

    headingError = wrapAngle(targetAngle-robotYaw);
    speed = baseSpeed;
    if abs(headingError) > 0.9
        speed = speed*0.45;
    end
    yawRate = max(-maxYawRate, min(maxYawRate, turnGain*headingError));
    % Stop as soon as the observation point has been reached.
    if goalDist < 1.6
        speed = min(speed, max(0.5, goalDist*0.8));
    end
    newYaw = wrapAngle(robotYaw + yawRate*dt);
    newXY = robotXY + speed*[cos(newYaw), sin(newYaw)]*dt;

    % Keep the rover inside the mapped world.
    margin = 0.8;
    newXY = max(worldMin+margin, min(worldMin+gridSize*cellSize-margin, newXY));
    mode = 'NORMAL';
end

function angle = wrapAngle(angle)
    angle = atan2(sin(angle), cos(angle));
end

function dmin = frontLidarDistance(robotXY, yaw, obstacles, numRays, maxDistance)
    dmin = maxDistance;
    for k = 1:numRays
        rel = deg2rad(-55 + 110*(k-1)/max(1,numRays-1));
        angle = yaw + rel;
        direction = [cos(angle), sin(angle)];
        for j = 1:size(obstacles,1)
            delta = obstacles(j,1:2) - robotXY;
            projection = dot(delta, direction);
            if projection <= 0
                continue;
            end
            perp2 = dot(delta,delta) - projection^2;
            radius = obstacles(j,3);
            if perp2 <= radius^2
                hit = projection - sqrt(max(0, radius^2-perp2));
                if hit >= 0
                    dmin = min(dmin, min(maxDistance, hit));
                end
            end
        end
    end
end

function nearest = updateLidarGraphics(handles, robotXY, yaw, obstacles, numRays, maxDistance)
    nearest = maxDistance;
    for k = 1:numRays
        rel = deg2rad(-55 + 110*(k-1)/max(1,numRays-1));
        angle = yaw + rel;
        direction = [cos(angle), sin(angle)];
        hitDistance = maxDistance;
        for j = 1:size(obstacles,1)
            delta = obstacles(j,1:2) - robotXY;
            projection = dot(delta, direction);
            if projection <= 0
                continue;
            end
            perp2 = dot(delta,delta) - projection^2;
            radius = obstacles(j,3);
            if perp2 <= radius^2
                hit = projection - sqrt(max(0, radius^2-perp2));
                if hit >= 0
                    hitDistance = min(hitDistance, hit);
                end
            end
        end
        nearest = min(nearest, hitDistance);
        endPoint = robotXY + hitDistance*direction;
        set(handles(k), 'XData', [robotXY(1) endPoint(1)], ...
            'YData', [robotXY(2) endPoint(2)], 'ZData', [0.78 0.78]);
        if hitDistance < 1.2
            set(handles(k), 'Color', [1 0.15 0.12], 'LineWidth', 1.5);
        else
            set(handles(k), 'Color', [0.76 0.82 0.72], 'LineWidth', 0.8);
        end
    end
end

function plotSimulationResults(t, dist, speed, mode, x, y, startXY, encounters, goalXY)
    if isempty(t)
        return;
    end
    resultFig = figure('Name', 'Wildlife Robot Simulation Results', 'Color', 'w');
    tiledlayout(resultFig, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

    nexttile;
    valid = isfinite(dist);
    plot(t(valid), dist(valid), 'LineWidth', 1.6);
    xlabel('Time (s)'); ylabel('Distance to observation goal (m)');
    title('Distance to Goal'); grid on;

    nexttile;
    plot(t, speed, 'LineWidth', 1.6);
    xlabel('Time (s)'); ylabel('Virtual speed (m/s)');
    title('Robot Speed'); grid on;

    nexttile([1 2]);
    plot(x, y, 'b-', 'LineWidth', 1.7); hold on;
    plot(startXY(1), startXY(2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
    if all(isfinite(goalXY))
        plot(goalXY(1), goalXY(2), 'p', 'MarkerFaceColor', [0.1 0.8 0.2], ...
            'MarkerEdgeColor', 'k', 'MarkerSize', 12);
    end
    if ~isempty(encounters)
        plot(encounters(:,1), encounters(:,2), 'ro', 'MarkerSize', 6, 'LineWidth', 1.2);
    end
    xlabel('World X (m)'); ylabel('World Y (m)');
    title('Robot Trajectory'); axis equal; grid on;
    legendItems = {'Robot path','Start','Observation goal','Obstacle/safety encounter'};
    legend(legendItems(1:(3+~isempty(encounters))), 'Location', 'best');

    % Report the final simulated controller state counts.
    if ~isempty(mode)
        labels = unique(mode);
        fprintf('\nSimulation log summary:\n');
        for k = 1:numel(labels)
            fprintf('  %-14s : %d frames\n', labels{k}, sum(strcmp(mode,labels{k})));
        end
    end
end

function keyPressCallback(src, event)
    if strcmpi(event.Key, 'q') || strcmpi(event.Key, 'escape')
        setappdata(src, 'quitRequested', true);
    elseif strcmpi(event.Key, 'r')
        setappdata(src, 'resetRequested', true);
    end
end

function closeCallback(src, ~)
    % Guard against callbacks firing while MATLAB is already deleting figures.
    if ~isgraphics(src)
        return;
    end
    try
        setappdata(src, 'quitRequested', true);
        % Hide instead of deleting here; the main loop will exit cleanly.
        set(src, 'Visible', 'off');
    catch
        % Ignore a figure that MATLAB has already started closing.
    end
end
