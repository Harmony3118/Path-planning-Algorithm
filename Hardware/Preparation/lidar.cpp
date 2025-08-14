#include "lidar.h"
#include <iostream>
#include <vector>
#include <fcntl.h>
#include <termio.h>
#include <unistd.h>
#include <cstring>

/*
/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\
I NEED TO KNOW THE MODEL OF THE LIDAR
/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\
*/
int lidar_fd = -1;

void initLidar()
{
    lidar_fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
    if (lidar_fd == -1)
    {
        exit(1);
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(lidar_fd, &tty) != 0)
    {
        exit(1);
    }

    cfsetospeed(&tty, B115200); // classic speed for RPLIDAR
    cfsetispeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD); // Activer réception
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;      // 8 bits
    tty.c_cflag &= ~PARENB;  // Pas de parité
    tty.c_cflag &= ~CSTOPB;  // 1 stop bit
    tty.c_cflag &= ~CRTSCTS; // Pas de contrôle de flux

    tcsetattr(lidar_fd, TCSANOW, &tty);
}

std::vector<float> getLidarData()
{
    std::vector<float> scan(360, 2.0f);

    char buffer[512];
    int n = read(lidar_fd, buffer, sizeof(buffer));
    if (n > 0)
    {
        // TODO: adapt this to the specific LIDAR model
        for (int i = 0; i < 360; i++)
        {
            scan[i] = 0.5f + 1.0f * (i % 30) / 30.0f;
        }
    }
    else
    {
        std::cerr << "[LIDAR] No data received";
    }
    return scan;
}