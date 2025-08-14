clc, close all, clear all

% cppFile = 'RRT_Algorithm.cpp';
% mexName = ['RRT_Algorithm.',mexext()];
cppFile = 'RRTstar_Algorithm.cpp';
mexName = ['RRTstar_Algorithm.',mexext()];

if ~isfile(mexName) || dir(cppFile).datenum > dir(mexName).datenum
    mex(cppFile);
end

start = [2.5 3.5 0];
goal = [2.5 0.2 0];

load exampleMaps % emptyMap, simpleMap and complexMap
cellPerMeter = 10;
map = occupancyMap(complexMap, cellPerMeter);

mapMatrix = occupancyMatrix(map); 

disp(start);
disp(goal);

% [path, explored] = RRT_Algorithm(start, goal, mapMatrix, cellPerMeter);
[path, explored] = RRTstar_Algorithm(start, goal, mapMatrix, cellPerMeter);

afficher_planification(map, path, explored, start, goal);
