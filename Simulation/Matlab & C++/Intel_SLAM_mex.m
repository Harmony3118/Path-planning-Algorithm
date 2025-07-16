clc, close all, clear all

cppFile = 'Intel_SLAM_Algorithm.cpp';
mexName = ['Intel_SLAM_Algorithm.',mexext()];

if ~isfile(mexName) || dir(cppFile).datenum > dir(mexName).datenum
    mex(cppFile);
end

if ~isfile('intelSlamData.mat')
    parseIntelLogFile('../Dataset_SLAM/intel.log');
end
load('intelSlamData.mat');

Intel_SLAM_Algorithm('offlineSlamData.mat');