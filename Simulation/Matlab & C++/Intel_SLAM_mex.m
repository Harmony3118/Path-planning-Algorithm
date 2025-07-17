clc, close all, clear all

cppFile = 'Intel_SLAM_Algorithm.cpp';
mexName = ['Intel_SLAM_Algorithm.',mexext()];

if ~isfile(mexName) || dir(cppFile).datenum > dir(mexName).datenum
    mex(cppFile, ...
        '-I/home/pauline/libs/eigen-3.4.0', ...
        '-I/usr/local/include/g2o', ...
        '-L/usr/local/lib/g2o', ...
        '-lg2o_core', ...
        '-lg2o_cli', ...
        '-lg2o_solver_cholmod', ...
        '-lg2o_stuff', ...
        '-lg2o_types_slam2d', ...
        '-lfmt');
end
disp("Compilation done");

if ~isfile('intelSlamData.mat')
    disp("Parse en cours");
    parseIntelLogFile('../Dataset_SLAM/intel.log');
end
load('intelSlamData.mat');
disp(".map load");

[posesInit, posesOpti] = Intel_SLAM_Algorithm('intelSlamData.mat');

figure;
plot(posesInit(:,1), posesInit(:,2), '-o');
quiver(posesInit(:,1), posesInit(:,2), cos(posesInit(:,3)), sin(posesInit(:,3)), 0.5);
xlabel('x');
ylabel('y');
axis equal;
title('Trajectoire initiale');

figure;
plot(posesOpti(:,1), posesOpti(:,2), '-o');
quiver(posesOpti(:,1), posesOpti(:,2), cos(posesOpti(:,3)), sin(posesOpti(:,3)), 0.5);
xlabel('x');
ylabel('y');
axis equal;
title('Trajectoire optimisée');