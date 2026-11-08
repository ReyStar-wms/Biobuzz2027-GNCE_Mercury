package org.firstinspires.ftc.teamcode.config.core.util.hardware;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Centralizes driver-gamepad rumble. Every rumble is a short one-shot effect (no continuous buzz), and
 * a hub command is only sent at the instant a pulse starts -- never per loop.
 *
 * Two cues:
 *  - FULL: a brief buzz fired ONCE when the magazine becomes full (rising edge), then silent. Not held.
 *  - SHOT: a short "balls launched" buzz that PREEMPTS the full buzz. Because the full buzz is one-shot
 *    it is normally finished by the time you shoot; if you fire right after loading, the shot buzz just
 *    overrides whatever is playing.
 *
 * Shot-cue timing. The goal is for the driver to FEEL the cue exactly as the balls clear, so they can
 * drive off immediately. {@link #triggerShot()} is called when the transfer starts pushing the balls;
 * the balls leave {@link #BALL_LAUNCH_OFFSET_MS} later, and a command takes {@link #HUB_RESPONSE_DELAY_MS}
 * to be felt, so we schedule the send for:
 *
 *   send = trigger + max(0, BALL_LAUNCH_OFFSET_MS - HUB_RESPONSE_DELAY_MS - EXPECTED_LOOP_MS/2)
 *
 * The half-loop term centers the (loop-quantized) send on the target instead of landing late. Human
 * reaction time is intentionally NOT subtracted: cuing before the balls are out would invite the driver
 * to move mid-shot. Reaction happens after balls-out (safe), and the cue lasts long enough to still be
 * buzzing when they react.
 */
@Configurable
public class RumbleManager {

    // Time from the transfer push starting to the balls physically clearing the shooter.
    public static long BALL_LAUNCH_OFFSET_MS = 110;
    // Time from sending a rumble command to the driver feeling it (DS + gamepad transport).
    public static long HUB_RESPONSE_DELAY_MS = 60;
    // Nominal loop period; half of it centers the (loop-quantized) send on the target instant.
    public static long EXPECTED_LOOP_MS = 40;

    // Shot cue: a single short buzz fired as the balls clear.
    public static long SHOT_RUMBLE_MS = 250;

    // "Magazine full" cue: a single short buzz fired once when it fills (not continuous).
    public static long FULL_RUMBLE_MS = 300;

    private final Gamepad gamepad;

    private boolean shotPending = false;
    private long shotSendAtMs = 0L;
    private long shotEndsAtMs = 0L;

    private boolean prevFull = false;

    public RumbleManager(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    private long shotDurationMs() {
        return SHOT_RUMBLE_MS;
    }

    /**
     * Call once at the instant the transfer starts pushing the balls out. Schedules the cue so it is
     * felt as the balls clear (see class doc). A second call within a shot just reschedules.
     */
    public void triggerShot() {
        long now = System.currentTimeMillis();
        long lead = Math.max(0L, BALL_LAUNCH_OFFSET_MS - HUB_RESPONSE_DELAY_MS - EXPECTED_LOOP_MS / 2L);
        shotSendAtMs = now + lead;
        shotPending = true;
    }

    /** Call every loop. {@code full} = the magazine-full condition (a one-shot buzz fires on its rising edge). */
    public void update(boolean full) {
        long now = System.currentTimeMillis();

        // Fire the scheduled shot cue once its send time arrives; it preempts any in-progress full buzz.
        if (shotPending && now >= shotSendAtMs) {
            shotPending = false;
            shotEndsAtMs = now + shotDurationMs();
            gamepad.rumble(1.0, 1.0, (int) SHOT_RUMBLE_MS);
        }

        boolean shotBusy = shotPending || now < shotEndsAtMs;

        // One short buzz on the rising edge of "full" -- not while the shot cue owns the motor.
        if (full && !prevFull && !shotBusy) {
            gamepad.rumble(1.0, 1.0, (int) FULL_RUMBLE_MS);
        }
        prevFull = full;
    }

    /** Forget any pending/active cue and edge state (e.g. on opmode (re)start). */
    public void reset() {
        shotPending = false;
        shotEndsAtMs = 0L;
        prevFull = false;
    }
}
