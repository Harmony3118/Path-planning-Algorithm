clc, close all, clear all

cppFile = 'Intel_SLAM_Algorithm.cpp';
mexName = ['Intel_SLAM_Algorithm.',mexext()];

if ~isfile(mexName) || dir(cppFile).datenum > dir(mexName).datenum
    mex('-I/home/pauline/libs/eigen-3.4.0', cppFile);
end
disp("Compilation done");

if ~isfile('intelSlamData.mat')
    disp("Parse en cours");
    parseIntelLogFile('../Dataset_SLAM/intel.log');
end
load('intelSlamData.mat');
disp(".map load");

Intel_SLAM_Algorithm('intelSlamData.mat');