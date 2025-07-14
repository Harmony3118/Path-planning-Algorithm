%% Adapt and parse the dataset 
clc; clear all; close all;

bag = rosbag('../Dataset_SLAM/2011-01-24-06-18-27.bag');

% bag.AvailableTopics

% Extract Laser scans
scanSel = select(bag,'Topic','/base_scan');
scanMsgs = readMessages(scanSel);
scans = cellfun(@(m) lidarScan(double(m.Ranges), double(m.AngleMin):double(m.AngleIncrement):double(m.AngleMax)), scanMsgs, 'UniformOutput', false);
disp('Data scan extract');
%%
% Extract Odometry
odomSel = select(bag,'Topic','/base_odometry/odom');
disp('odomSel ok');
odomMsgs = readMessages(odomSel);
disp('odomMsgs ok');

l = length(odomMsgs);
disp('l' + l);
poses = zeros(l, 3);
for i = 1:l
    pos = odomMsgs{i}.Pose.Pose.Position;
    ori = odomMsgs{i}.Pose.Pose.Orientation;
    eul = quat2eul([ori.W ori.X ori.Y ori.Z]);
    poses(i,:) = [pos.X pos.Y eul(1)];
end
disp('Data odometry extract');

% Synchronisation scans and odometry ?

save('offlineSlamData.mat', 'scans', 'poses');
