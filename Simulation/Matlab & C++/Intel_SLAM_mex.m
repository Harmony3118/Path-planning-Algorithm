clc, close all, clear all

if ~isfile('intelSlamData.mat')
    disp("Parse en cours");
    Intel_parseLogFile('../Dataset_SLAM/intel.log');
end
load('intelSlamData.mat');
disp(".map load");

%%

N = 13631;
fid = fopen('positions.csv', 'w');

for i = 1:N
    tmp = scanStruct(i).pose;
    fprintf(fid, '%.15f;%.15f;%.15f\n', tmp(1), tmp(2), tmp(3));
end

fclose(fid);

%%
clc;
cppFile = 'Intel_SLAM_Algorithm.cpp';
mexName = ['Intel_SLAM_Algorithm.',mexext()];

if ~isfile(mexName) || dir(cppFile).datenum > dir(mexName).datenum
    mex(cppFile, ...
        'CXXFLAGS="\$CXXFLAGS -fopenmp"', 'LDFLAGS="\$LDFLAGS -fopenmp"', ...
        '-I/home/pauline/libs/eigen-3.4.0', ...
        '-I/usr/local/include/g2o', ...
        '-I/usr/local/include/pointmatcher', ...
        '-I/usr/local/inclu de/nabo', ...
        '-I/usr/local/include/yaml-cpp', ... 
        '-L/usr/local/lib/g2o', ...
        '-L/usr/local/lib', ...
        '-lg2o_core', ...
        '-lg2o_cli', ...
        '-lg2o_solver_cholmod', ...
        '-lg2o_stuff', ...
        '-lg2o_types_slam2d', ...
        '-lfmt', ...
        '-lpointmatcher', ...
        '-lboost_system', ...
        '-lboost_filesystem', ...
        '-lyaml-cpp', ...
        '-lnabo'...
        );
end
disp("Compilation done");

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