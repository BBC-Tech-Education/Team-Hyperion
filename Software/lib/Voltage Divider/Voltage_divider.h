/**
 * @file Voltage_divider.h
 * @brief Battery pack voltage measurement via resistive divider + ADC.
 *
 * Converts analogRead() into volts using:
 *   V = (ADC / divider) + offset
 * where @c divider absorbs the divider ratio and ADC full-scale scaling, and
 * @c offset corrects systematic bias.
 *
 * @author S.Garg (Brisbane Boys' College)
 */

#ifndef VOLTDIV_H
#define VOLTDIV_H

class VoltageDivider {
public:
    /**
     * @param p ADC GPIO pin.
     * @param d Scale factor (ADC counts → volts denominator).
     * @param o Additive offset (volts).
     */
    VoltageDivider(uint8_t p, float d, float o) : pin(p), divider(d), offset(o) {}

    void init();

    /** @return Estimated pack voltage (V). */
    float get_lvl();

private:
    uint8_t pin;
    float divider;
    float offset;
};

#endif
