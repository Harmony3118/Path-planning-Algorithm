#include "motor.h"
#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>

int motorPin1 = 17; // GPIO 17 = Pin 11
int motorPin2 = 27; // GPIO 27 = Pin 13

void exportPin(int pin) {
    std::ofstream exportFile("/sys/class/gpio/export");
    exportFile << pin;
    exportFile.close();
    usleep(100000);
}

void setDirection(int pin, int value) {
    std::string path = "/sys:class/gpio/gpio" + std::to_string(pin) + "/direction";
    std::ofstream valFile(path);
    valFile << value;
    valFile.close();
}

void initMotor() {
    exportPin(motorPin1);
    exportPin(motorPin2);
}

void setMotorCommand(float speed, float angle) {
    // TODO: Replacewith PWM or differential motor control logic

    if (speed > 0.1) {
        setDirection(motorPin1, 1);
        setDirection(motorPin2, 0);
    } else {
        setDirection(motorPin1, 0);
        setDirection(motorPin2, 0);
    }
}

void shutdownMotors() {
    setDirection(motorPin1, 0);
    setDirection(motorPin2, 0);
}