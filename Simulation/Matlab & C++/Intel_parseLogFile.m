function scanStruct = Intel_parseLogFile(logfile)
    
    fid = fopen(logfile, 'r');
    if fid == -1
        error('Impossible d''ouvrir le fichier %s', logfile);
    end

    scanStruct = struct('ranges', [], 'angles', [], 'cartesian', [], 'count', [], 'pose', []);
    scanIdx = 1;

    while ~feof(fid) % tant que le fichier est ouvert
        line = fgetl(fid);

        if startsWith(line, 'FLASER')
            % FLASER num_readings [range_readings] x y theta odom_x odom_y
            % odom_theta timestamp hostname logger_time
            % odom_x odom_y odom_theta = same data as ODOM lign
            tokens = strsplit(line);
            num_readings = str2double(tokens{2});

            x = str2double(tokens{186}); % 2+num_readings+3+ 1
            y = str2double(tokens{187}); % 2+num_readings+3+ 2
            theta = str2double(tokens{188}); % 2+num_readings+3+ 3
            currentPose = [x, y, theta];

            % Ranges
            rangeStartIdx = 3;
            rangeEndIdx = rangeStartIdx + num_readings - 1;
            ranges = str2double(tokens(rangeStartIdx:rangeEndIdx));
            
            % Angles
            % By agreement : scanning π Rad from -π/2 to π/2 Rad
            angleMin = -pi/2;
            angleMax = pi/2;
            angleIncr = pi / (num_readings - 1);
            angles = angleMin : angleIncr : angleMax;

            % Cartesian
            [X, Y] = pol2cart(angles, ranges);
            cartesian = [X(:), Y(:)];

            % Créer struct Scan
            scan.ranges = ranges(:);
            scan.angles = angles;
            scan.cartesian = cartesian;
            scan.count = num_readings;
            scan.pose = currentPose;

            scanStruct(scanIdx) = scan;
            disp(scanIdx);
            scanIdx = scanIdx + 1;
        end
    end
    
    fclose(fid);

    save('intelSlamData.mat', 'scanStruct');
    clear all;
end