clear all; clc; close all;

load exampleMaps % emptyMap, simpleMap and complexMap
cellPerMeter = 10;
map = occupancyMap(simpleMap, cellPerMeter); % 10 cell per meter
% robotRadius = 0.2;
% mapInflated = copy(map);
% inflate(mapInflated,robotRadius);

ss = stateSpaceSE2; % [x, y, θ]
sv = validatorOccupancyMap(ss);
sv.Map = map;
sv.ValidationDistance = 0.05; % ??
ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];

start = [0.2 0 0];
goal = [0.9 0.4 0];

function h = heuristicSE2(state, goal)
    dx = state(1) - goal(1);
    dy = state(2) - goal(2);
    dtheta = wrapToPi(state(3) - goal(3));

    h = sqrt(dx^2 + dy^2) + abs(dtheta)*0.1;
end

function neighbors = getNeighbors(state, cellPerMeter)
    d = 1/cellPerMeter;
    dtheta = pi/2;

    x = state(1) + d*cos(state(3));
    y = state(2) + d*sin(state(3));
    tetha = state(3);

    neighbors = [
        x, y, tetha;
        x, y, tetha+dtheta; % turn right before moving
        x, y, tetha-dtheta; % turn left before moving
    ];    
end

openList = [];
closeList = [];

startNode.State = start;
startNode.g = 0;
startNode.h = heuristicSE2(start, goal);
startNode.poids = startNode.g + startNode.h;
startNode.Parent = [];

disp(startNode);

openList = [openList, startNode];
[zztest, idx] = min([openList.poids]);

while ~isempty(openList)
    [~, idx] = min([openList.poids]);
    current = openList(idx);
    openList(idx) = [];
    disp(current);

    if norm(current.State(1:2) - goal(1:2)) < 0.1
        disp("Goal reached!");
        break;
    end

    neighbors = getNeighbors(current.State, cellPerMeter);

    for i=1:size(neighbors, 1)
        neighborState = neighbors(i,:);

        % if the state is valid
        if ~isStateValid(sv, neighborState)
            continue;
        end
        % is state is in closeList
        if any(arrayfun(@(n) isequal(round(n.State,2), ...
                                     round(neighborState,2)), ...
                        closeList))
            continue;
        end
        newNode.State = neighborState;
        % ou mais d=0.1
        newNode.g = current.g + norm(current.State(1:2) - neighborState(1:2));
        newNode.h = heuristicSE2(neighborState, goal);
        newNode.poids = newNode.g + newNode.h;
        newNode.Parent = current;

        openList = [openList; newNode];
    end
    closeList = [closeList; current];

    path = current.State;
    while ~isempty(current.Parent)
        current = current.Parent;
        path = [current.State; path];
    end
end

figure;
show(map);
hold on;
plot(path(:,1), path(:,2), 'r-', 'LineWidth', 2);


% 
% rng = ('shuffle');
% 

% %h = ?
% color = zeros(sizeMap);
% disp(color);
% disp(map.)
% 
% %show(map);
% 
% %xRnd = rng(20);
% %yRnd = 