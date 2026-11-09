package org.firstinspires.ftc.teamcode.opmodes.auto.base;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * Checks the three storage beam breaks. Nothing moves.
 *
 * <p>{@code Storage} drives p1-p3 as outputs to power the emitters and reads s1-s3 as inputs. The
 * decisive test is here: with the emitters powered the receivers should read one state, and with
 * them switched off they should all read the other. A receiver that does not follow its emitter is
 * mis-wired or dead - and because {@code Storage} counts artifacts off these beams, a stuck one
 * silently breaks the intake cutoff and the shot sequence.
 */
@Autonomous(name = "06 Check: storage beam breaks", group = "Device Check")
public class StorageSensorCheck extends DeviceCheckOpMode {

    @Override
    protected void runChecks() {
        section("Devices");
        DigitalChannel[] receivers = new DigitalChannel[Devices.BEAM_RECEIVERS.length];
        DigitalChannel[] emitters = new DigitalChannel[Devices.BEAM_EMITTERS.length];
        for (int i = 0; i < receivers.length; i++) {
            receivers[i] = device(DigitalChannel.class, Devices.BEAM_RECEIVERS[i]);
        }
        for (int i = 0; i < emitters.length; i++) {
            emitters[i] = device(DigitalChannel.class, Devices.BEAM_EMITTERS[i]);
        }
        for (DigitalChannel channel : receivers) {
            if (channel == null) {
                return;
            }
        }
        for (DigitalChannel channel : emitters) {
            if (channel == null) {
                return;
            }
        }

        section("Modes");
        check("set directions", () -> {
            for (DigitalChannel receiver : receivers) {
                receiver.setMode(DigitalChannel.Mode.INPUT);
            }
            for (DigitalChannel emitter : emitters) {
                emitter.setMode(DigitalChannel.Mode.OUTPUT);
            }
            return "s1-s3 input, p1-p3 output (same as Storage)";
        });

        section("Emitters on");
        final boolean[] lit = new boolean[receivers.length];
        check("read with emitters powered", () -> {
            setEmitters(emitters, true);
            sleep(400);
            for (int i = 0; i < receivers.length; i++) {
                lit[i] = receivers[i].getState();
            }
            return states(lit);
        });

        section("Emitters off");
        final boolean[] dark = new boolean[receivers.length];
        check("read with emitters off", () -> {
            setEmitters(emitters, false);
            sleep(400);
            for (int i = 0; i < receivers.length; i++) {
                dark[i] = receivers[i].getState();
            }
            return states(dark);
        });

        check("every receiver follows its emitter", () -> {
            StringBuilder stuck = new StringBuilder();
            for (int i = 0; i < receivers.length; i++) {
                if (lit[i] == dark[i]) {
                    stuck.append(Devices.BEAM_RECEIVERS[i])
                            .append(" stuck ").append(lit[i] ? "true" : "false").append("; ");
                }
            }
            require(stuck.length() == 0, stuck.toString()
                    + "- that receiver is not seeing its emitter (wiring, or the pair is dead)");
            return "all three changed state";
        });

        // Put the emitters back the way Storage leaves them before the live check.
        check("restore emitters", () -> {
            setEmitters(emitters, true);
            sleep(300);
            return "p1-p3 powered";
        });

        section("Live");
        observe("block each beam with a finger in turn", 12000, () -> {
            boolean[] now = new boolean[receivers.length];
            for (int i = 0; i < receivers.length; i++) {
                now[i] = receivers[i].getState();
            }
            return states(now);
        });
    }

    private static void setEmitters(DigitalChannel[] emitters, boolean on) {
        for (DigitalChannel emitter : emitters) {
            emitter.setState(on);
        }
    }

    private static String states(boolean[] values) {
        StringBuilder out = new StringBuilder();
        for (int i = 0; i < values.length; i++) {
            if (i > 0) {
                out.append("  ");
            }
            out.append(Devices.BEAM_RECEIVERS[i]).append('=').append(values[i]);
        }
        return out.toString();
    }
}
