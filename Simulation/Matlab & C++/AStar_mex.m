clc, close all, clear all

cppFile = 'Astar_Algorithm_v2.cpp';
mexName = ['Astar_Algorithm_v2.',mexext()];

if ~isfile(mexName) || dir(cppFile).datenum > dir(mexName).datenum
    mex(cppFile);
end

start = [1.1 2.2 0];
goal = [2.5 0.2 0];

load exampleMaps % emptyMap, simpleMap and complexMap
cellPerMeter = 10;
map = occupancyMap(simpleMap, cellPerMeter);

mapMatrix = occupancyMatrix(map); 

[path, explored] = Astar_Algorithm_v2(start, goal, mapMatrix, cellPerMeter);

afficher_planification(map, path, explored, start, goal);

% % === 3. Affichage de la carte ===
% figure;
% show(map)
% hold on
% title('Chemin du robot mobile')
% 
% % === 4. Tracé du chemin ===
% scatter(explored(:,1), explored(:,2), 20, 'b', 'filled')
% quiver(explored(:,1), explored(:,2), cos(explored(:,3)), sin(explored(:,3)), 0.3, 'k')
% plot(path(:,1), path(:,2), 'r-', 'LineWidth', 2)
% scatter(path(1,1), path(1,2), 80, 'go', 'filled') % Point de départ
% scatter(path(end,1), path(end,2), 80, 'ro', 'filled') % But
