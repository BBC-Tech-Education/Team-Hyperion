/**
 * @file Light_system.h
 * @brief Multiplexed reflectance line array → robot-relative line angle/size.
 *
 * Hardware: three analog mux outputs (LS_OUT_1..3) addressed by a shared
 * 4-bit select bus. muxIndex[] remaps mux channel order into a contiguous
 * angular sensor index (inner ring + outer corner sensors).
 *
 * Outputs:
 *   lineDir  — robot-frame direction toward the white line (−1 if none)
 *   lineSize — penetration depth proxy in roughly [0, 1+] (−1 if none)
 */

#ifndef LIGHT_SYSTEM_H
#define LIGHT_SYSTEM_H

#include "Pins.h"
#include "Config.h"

class LightSystem {
public:
    /** Configure mux GPIO, capture dark-field baselines → thresholds[]. */
    void init();

    /** Sample sensors, threshold, cluster, and update lineDir/lineSize. */
    void update();

    float get_line_angle(); /**< @return Robot-frame line direction (deg) or −1. */
    float get_line_size();  /**< @return Line size proxy or −1. */

private:
    /** Sweep all mux addresses and fill sensorReadings[] via muxIndex[]. */
    void read();

    /** Contiguous block of sensors seeing white on the inner ring. */
    struct Cluster {
        int8_t start = -1;      /**< First sensor index in cluster. */
        float midpoint = -1.0f; /**< Angular midpoint of cluster (deg). */
        int8_t end = -1;        /**< Last sensor index in cluster. */
    };

    uint8_t inPin[4] = {LS_IN_0, LS_IN_1, LS_IN_2, LS_IN_3}; /**< Mux address bits. */

    /**
     * Physical→logical remapping for 48 channels across 3 muxes.
     * Indices 0..LS_INNER_NUM-1 form the circular inner ring (11.25°/sensor).
     * Higher indices are outer corner groups used as fallback.
     */
    uint8_t muxIndex[LS_NUM] = {
        40, 14, 41, 13, 42, 12, 43, 11, 15, 10, 16, 39, 17, 38, 18,  9, // Mux 1
         3,  4,  5,  6, 36, 37,  7,  8, 35, 34, 33, 32,  2,  1,  0, 31, // Mux 2
        24, 30, 20, 26, 22, 28, 44, 47, 23, 29, 19, 25, 21, 27, 45, 46  // Mux 3
    };

    uint16_t thresholds[LS_NUM];    /**< Per-sensor white threshold (ADC). */
    uint16_t sensorReadings[LS_NUM]; /**< Latest ADC samples. */
    uint8_t onWhite[LS_NUM];        /**< Binary white detection after fill-in. */

    uint8_t clusterNum; /**< Number of contiguous white clusters found. */
    bool inCluster;     /**< Cluster-scan state flag. */

    float lineDir;  /**< Published robot-frame line angle. */
    float lineSize; /**< Published line size. */
};

#endif
