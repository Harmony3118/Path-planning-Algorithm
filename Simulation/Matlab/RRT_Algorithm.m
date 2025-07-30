%%
clc; clear all; close all;

ss = stateSpaceSE2; % 2D pose space [x, y, θ]
sv = validatorOccupancyMap(ss); % check if a state is valid, not in obstacle
load exampleMaps % simpleMap and complexMap
map = occupancyMap(simpleMap, 10); 
sv.Map = map; % link map and validator

sv.ValidationDistance = 0.01; % validator chek all 0.01 meters states
ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];

planner = plannerRRT(ss, sv, MaxConnectionDistance=0.3);
start = [1.1 2.2 0];
goal = [2.5 0.2 0];

rng('shuffle'); % random initialize by time machine
[pthObj,solnInfo] = plan(planner, start, goal);
% pthObj = path object with the states along the path
% solnInfo = additional data, like tree structure and planning stats.

show(map)
hold on
% Tree expansion
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-')
% Draw path
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2)