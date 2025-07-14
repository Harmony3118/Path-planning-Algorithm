clc, close all, clear all

cppFile = 'SLAM_Algorithm.cpp';
mexName = ['SLAM_Algorithm.',mexext()];

if ~isfile(mexName) || dir(cppFile).datenum > dir(mexName).datenum
    mex(cppFile);
end

if ~isfile('offlineSlamData.mat')
    disp('Parse data, it can take some time');
    bag = rosbag('../Dataset_SLAM/2011-01-24-06-18-27.bag');
    
    % Extract Laser scans simple to insert in c++ code
    scanSel = select(bag,'Topic','/base_scan');
    scanMsgs = readMessages(scanSel);

    numScans = length(scanMsgs);
    scanStruct(numScans) = struct('ranges', [], 'angles', [], 'cartesian', [], 'count', [], 'pose', []);

    % Extract Odometry
    odomSel = select(bag,'Topic','/base_odometry/odom');
    odomMsgs = readMessages(odomSel);
    
    l = length(odomMsgs);
    poses = zeros(l, 3);
    for i = 1:l
        pos = odomMsgs{i}.Pose.Pose.Position;
        ori = odomMsgs{i}.Pose.Pose.Orientation;
        eul = quat2eul([ori.W ori.X ori.Y ori.Z]);
        poses(i,:) = [pos.X pos.Y eul(1)];
    end
    
    % Synchronisation scans and odometry
    scanTimes = cellfun(@(m) double(m.Header.Stamp.Sec) + double(m.Header.Stamp.Nsec)*1e-9, scanMsgs);
    odomTimes = cellfun(@(m) double(m.Header.Stamp.Sec) + double(m.Header.Stamp.Nsec)*1e-9, odomMsgs);

    matchedPoses = zeros(length(scanTimes),3);
    for i = 1:length(scanTimes)
        % Find the closest odometry time index
        [~,idx] = min (abs(odomTimes - scanTimes(i)));
        matchedPoses(i, :) = poses(idx, :);
    end

    for i=1:numScans
        msg = scanMsgs{i};
        ranges = double(msg.Ranges(:));
        angleMin = double(msg.AngleMin);
        angleInc = double(msg.AngleIncrement);
        angles = angleMin + (0:length(ranges)-1)' * angleInc;

        x = ranges .* cos(angles);
        y = ranges .* sin(angles);
        cartesian = [x, y];

        scanStruct(i).ranges = ranges;
        scanStruct(i).angles = angles;
        scanStruct(i).cartesian = cartesian;
        scanStruct(i).count = length(ranges);
        scanStruct(i).pose = matchedPoses(i,:);
    end

    % Save all in one file
    save('offlineSlamData.mat', 'scanStruct', 'matchedPoses', 'odomTimes');
    clear all;
end
load('offlineSlamData.mat');

SLAM_Algorithm('offlineSlamData.mat');