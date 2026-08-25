/*!
 * @file pins.h
 * 
 * This is a library full of the pin numbers for each aspect of the robot.
 * 
 * S.Garg (Brisbane Boys' College)
 * T.McCabe (Brisbane Boys' College)
 */
#ifndef PINS_H
#define PINS_H

#include <Arduino.h>

//! @def FRPWM @brief Front Right Motor Power Pin
#define FRPWM 25
//! @def FRINA @brief Front Right Motor Input A
#define FRINA 26
//! @def FRINB @brief Front Right Motor Input B
#define FRINB 27
//! @def FLPWM @brief Front Left Motor Power Pin
#define FLPWM 11
//! @def FLINA @brief Front Left Motor Input A
#define FLINA 12
//! @def FLINB @brief Front Left Motor Input B
#define FLINB 24
//! @def BRPWM @brief Back Right Motor Power Pin
#define BRPWM 28
//! @def BRINA @brief Back Right Motor Input A
#define BRINA 29
//! @def BRINB @brief Back Right Motor Input B
#define BRINB 30
//! @def BLPWM @brief Back Left Motor Power Pin
#define BLPWM 8
//! @def BLINA @brief Back Left Motor Input A
#define BLINA 9
//! @def BLINB @brief Back Left Motor Input B
#define BLINB 10

//! @def TSSP1 @brief Tssp Pin
#define TSSP1 16
//! @def TSSP2 @brief Tssp Pin
#define TSSP2 13
//! @def TSSP3 @brief Tssp Pin
#define TSSP3 41
//! @def TSSP4 @brief Tssp Pin
#define TSSP4 40
//! @def TSSP5 @brief Tssp Pin
#define TSSP5 39
//! @def TSSP6 @brief Tssp Pin
#define TSSP6 37
//! @def TSSP7 @brief Tssp Pin
#define TSSP7 36
//! @def TSSP8 @brief Tssp Pin
#define TSSP8 33
//! @def TSSP9 @brief Tssp Pin
#define TSSP9 31
//! @def TSSP10 @brief Tssp Pin
#define TSSP10 7
//! @def TSSP11 @brief Tssp Pin
#define TSSP11 6
//! @def TSSP12 @brief Tssp Pin
#define TSSP12 5
//! @def TSSP13 @brief Tssp Pin
#define TSSP13 4
//! @def TSSP14 @brief Tssp Pin
#define TSSP14 3
//! @def TSSP15 @brief Tssp Pin
#define TSSP15 2
//! @def TSSP16 @brief Tssp Pin
#define TSSP16 17

//! @def LIGHT_PIN @brief Light Sensor Pin
#define LIGHT_PIN 21
//! @def LIGHT_PIN2 @brief Light Sensor Pin
#define LIGHT_PIN2 20
//! @def LIGHT_PIN_DIGI_0 @brief Light Sensor Pin in correspondance to Mux
#define LIGHT_PIN_DIGI_0 23
//! @def LIGHT_PIN_DIGI_1 @brief Light Sensor Pin in correspondance to Mux
#define LIGHT_PIN_DIGI_1 22
//! @def LIGHT_PIN_DIGI_2 @brief Light Sensor Pin in correspondance to Mux
#define LIGHT_PIN_DIGI_2 15
//! @def LIGHT_PIN_DIGI_3 @brief Light Sensor Pin in correspondance to Mux
#define LIGHT_PIN_DIGI_3 14

//! @def BATT_READ_PIN @brief Pin used to read battery of robot LiPo
#define BATT_READ_PIN 38


//! @def GOAL_PIN @brief The logic pin has various uses, it is connected to a
//!                       switch which is connected to two different conditions
//!                       depending on what is happening inside the code.
#define GOAL_PIN 32

#define COM_PIN 35

#endif