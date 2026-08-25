/*!
 * @file bluetooth.h
 * 
 * @mainpage This is a library for the HC-05 Bluetooth Module
 * 
 * @date 30/04/25
 * 
 * @author S.Garg (Brisbane Boys' College)
 */
#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <Arduino.h>
#include <Common.h>
#include <Config.h>
#include <Timer.h>


struct robotData {
    bool role;
    float ballDir;
    uint8_t ballStr;
    bool enabled;
};

/*!
 * @brief Class that stores state and functions for interacting with the HC-05 
 *        Bluetooth Module
 */
class Bluetooth {
public:
    Bluetooth() {};
    void init();
    void update(float ballDir, float ballStr, bool enabled);
    bool getRole();
private:
    void read();
    void send();

    robotData self = {false, 0.0, 0};
    robotData other = {false, 0.0, 0};
    Timer connectedTimer = Timer(1000000);
    Timer roleConflict = Timer(1000000);
    Timer sendTimer = Timer(10000);
    Timer switchTimer = Timer(1000000);

    bool otherPrevRole = false;
    bool switching = false;
};/*




    void send(bool logic, float ballDir, float ballStr);
    bool switching();
    //! @brief Location of the ball for the connected robot. [1] is ballDir, [2]
    //!        is ballStr
    float otherRobotBallLocation[2] = {0,0};
    //! @brief If the other robot is attacking or defending.
    float otherRobotLogic = 0;
    //! @brief If the robot is connected to another bluetooth module
    bool connection;
    
    bool read();
    Timer switchTimer = Timer(200000);
    bool read_data;
    int bluetoothBuffer[BLUETOOTH_PACKET_SIZE - 1];
    unsigned long last_sent_time;
    unsigned long last_received_time;
};*/


#endif