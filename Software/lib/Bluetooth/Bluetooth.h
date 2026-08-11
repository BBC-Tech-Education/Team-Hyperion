/**
 * @file Bluetooth.h
 * @brief Peer-to-peer role negotiation over UART Bluetooth link.
 *
 * Exchanges a compact score / ball strength / battery / enable packet with the
 * teammate robot and decides attacker vs defender (self.role).
 *
 * Packet (BT_PACKET_SIZE):
 *   [START][START][flags][score][ballStr][batLvl]
 * flags bit0 = role, bit1 = enabled.
 *
 * Link liveness: connectedTimer refreshed on every valid RX; timeout ⇒ offline.
 */

#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "Config.h"
#include "Timer.h"

/** Mirrored state for local robot or remote peer. */
struct PeerData {
    bool role = false;     /**< true = attacker (team convention in resolve_role). */
    bool enabled = false;  /**< Motors/COM enabled on that robot. */
    uint8_t score = 0;     /**< Suitability score for attacking. */
    uint8_t ballStr = 0;   /**< Ball strength / proximity. */
    uint8_t batLvl = 0;    /**< Battery level (scaled). */
};

class Bluetooth {
public:
    void init();

    /**
     * @brief One negotiation tick: latch local telemetry, RX, resolve, TX.
     * @param score   Local attack suitability (see compute_bt_role_score).
     * @param ballStr Local ball strength.
     * @param enabled Local enable/COM state.
     * @param batLvl  Local battery telemetry byte.
     */
    void update(uint8_t score, uint8_t ballStr, bool enabled, uint8_t batLvl);

    bool get_role() const { return self.role; }
    bool is_other_connected() const { return connected; }

private:
    void read();         /**< Parse inbound framed packets; refresh link timer. */
    void send();         /**< Emit local PeerData as a framed packet. */
    void resolve_role(); /**< Apply priority rules to set self.role. */

    PeerData self;
    PeerData other;

    Timer connectedTimer = Timer(BT_CONNECTION_TIMEOUT_US);

    bool switching = false; /**< True when peer flipped role while roles collided. */
    bool connected = false;
    bool otherPrevRole = false;
};

#endif
