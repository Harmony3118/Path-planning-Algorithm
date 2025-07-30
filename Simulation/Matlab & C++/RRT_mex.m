clc, close all, clear all

cppFile = 'RRT_Algorithm.cpp';
mexName = ['RRT_Algorithm.',mexext()];

if ~isfile(mexName) || dir(cppFile).datenum > dir(mexName).datenum
    mex(cppFile);
end

start = [1.1 2.2 0];
goal = [1.5 0.2 0];

load exampleMaps % emptyMap, simpleMap and complexMap
cellPerMeter = 10;
map = occupancyMap(simpleMap, cellPerMeter);

mapMatrix = occupancyMatrix(map); 

[path, explored] = RRT_Algorithm(start, goal, mapMatrix, cellPerMeter);

afficher_planification(map, path, explored, start, goal);
