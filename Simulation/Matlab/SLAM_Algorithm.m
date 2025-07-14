%%
clc; clear all; close all;
load('offlineSlamData.mat');
maxRange = max(scans{12}.Ranges);
disp(maxRange);

maxRangeAll = max(cellfun(@(s) max(s.Ranges), scans));

maxLidarRange = 0.90 * maxRangeAll; % meters, slightly less than the max
mapResolution = 20; % cells per meter (1 cell = 5cm)
slamAlg = lidarSLAM(mapResolution, maxLidarRange);

slamAlg.LoopClosureThreshold = 210;  
slamAlg.LoopClosureSearchRadius = 8;

% 10 first scan
for i=1:10
    %[isScanAccepted, ~, ~] = addScan(slamAlg, scans{i});
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scans{i});
    if isScanAccepted
        fprintf('Added scan %d \n', i);
    end
end

figure;
show(slamAlg);
title({'Map of the Environment','Pose Graph for Initial 10 Scans'});
%%
% first loop closure
firstTimeLCDetected = false;

figure;
for i=11:length(scans)
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scans{i});
    if ~isScanAccepted
        continue;
    end
    % visualize the first detected loop closure, if you want to see the
    % complete map building process, remove the if condition below
    if optimizationInfo.IsPerformed && ~firstTimeLCDetected
        show(slamAlg, 'Poses', 'off');
        hold on;
        show(slamAlg.PoseGraph); 
        hold off;
        firstTimeLCDetected = true;
        drawnow
    end
end
title('First loop closure');

figure
show(slamAlg);
title({'Final Built Map of the Environment', 'Trajectory of the Robot'});