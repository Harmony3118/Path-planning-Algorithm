#include "lidar.h"
#include "motor.h"
#include "utils.h"
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>

int main()
{
    initMotors();
    initLidar();

    while (true)
    {
        // Get LIDAR data
        std::vector<float> lidarData = getLidarScan();

        // algo here. While waiting mock data :
        float desiredSpeed = 0.5;
        float desiredAngle = 0.0;

        if (isObstacleTooClose(lidarData))
        {
            desiredSpeed = 0.0;
        }

        // Set motor command
        setMotorCommand(desiredSpeed, desiredAngle);

        // Sleep for a short duration to simulate control loop
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}