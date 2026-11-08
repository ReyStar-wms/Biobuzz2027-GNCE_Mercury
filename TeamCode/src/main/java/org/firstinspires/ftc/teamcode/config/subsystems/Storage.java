package org.firstinspires.ftc.teamcode.config.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.config.core.SubsysCore;
import org.firstinspires.ftc.teamcode.config.core.util.hardware.ModeSmoother;

@Configurable
public class Storage extends SubsysCore {
    private final DigitalChannel s1, s2, s3;
    private final DigitalChannel p1, p2, p3;

    private final ModeSmoother<Integer> beam1Smoother;
    private final ModeSmoother<Integer> beam2Smoother;
    private final ModeSmoother<Integer> beam3Smoother;

    private boolean st1, st2, st3;
    private int storedCount = 0;

    // Hard "full" cutoff: counts consecutive loops s1 (the last/full slot) reads broken.
    private int s1FullLoops = 0;
    private boolean s1HardFull = false;

    public static int beamSmoothWindow = 3;

    // If s1 (the LAST/full-slot beam) reads broken for this many consecutive loops, force a hard
    // "full" and cut the intake -- independent of the s2/s3 sequence count, which can stall when a
    // beam misses a ball. s1 being broken means the last slot is occupied, so the magazine is
    // physically full regardless of what the count thinks. Tune live; lower = faster cutoff but more
    // vulnerable to a transient, higher = safer but slower to react.
    public static int s1FullHoldLoops = 14;

    public static double currentLimit = 4;
    public static double veloLimit = 40;

    private double curAmps = 0;
    private double curVelo = 0;

    Timer timer;

    public Storage() {
        s1 = h.get(DigitalChannel.class, "s1");
        s2 = h.get(DigitalChannel.class, "s2");
        s3 = h.get(DigitalChannel.class, "s3");

        p1 = h.get(DigitalChannel.class, "p1");
        p2 = h.get(DigitalChannel.class, "p2");
        p3 = h.get(DigitalChannel.class, "p3");

        s1.setMode(DigitalChannel.Mode.INPUT);
        s2.setMode(DigitalChannel.Mode.INPUT);
        s3.setMode(DigitalChannel.Mode.INPUT);

        p1.setMode(DigitalChannel.Mode.OUTPUT);
        p2.setMode(DigitalChannel.Mode.OUTPUT);
        p3.setMode(DigitalChannel.Mode.OUTPUT);

        p1.setState(true);
        p2.setState(true);
        p3.setState(true);

        beam1Smoother = new ModeSmoother<>(beamSmoothWindow, 0);
        beam2Smoother = new ModeSmoother<>(beamSmoothWindow, 0);
        beam3Smoother = new ModeSmoother<>(beamSmoothWindow, 0);

        storedCount = 0;
        storageState = StorageState.WAIT_FOR_ST3;

        timer = new Timer();
    }

    public void input(double curAmps, double curVelo) {
        this.curAmps = curAmps;
        this.curVelo = Math.abs(curVelo);
    }

    public void clear() {
        storedCount = 0;
        storageState = StorageState.WAIT_FOR_ST3;
        s1FullLoops = 0;
        s1HardFull = false;
    }

    public int getSize() {
        return storedCount;
    }

    /**
     * True when the magazine is full. This is the OR of the normal sequence count (>= 3) and the
     * s1 hard-full cutoff, so a stalled count (e.g. from a faulty s2) can't keep the intake running
     * once the last slot is physically occupied. Gate the intake on THIS, not on getSize() == 3.
     */
    public boolean isFull() {
        return storedCount >= 3 || s1HardFull;
    }

    /** True only via the s1 hard-full cutoff (for telemetry / debugging the faulty-beam case). */
    public boolean isS1HardFull() {
        return s1HardFull;
    }

    @Override
    public void periodic() {
        readSmoothedBeams();
        updateStoredCount();

        if(telemetryEnabled){
            t.addData("Stored Count", storedCount);
            t.addData("Break Beam 1", st1);
            t.addData("Break Beam 2", st2);
            t.addData("Break Beam 3", st3);
            t.addData("S1 Full Loops", s1FullLoops);
            t.addData("S1 Force Full", s1HardFull);
            t.addData("Intake Full?", isFull());
        }
    }

    private void readSmoothedBeams() {
        beam1Smoother.add(s1.getState() ? 0 : 1);
        beam2Smoother.add(s2.getState() ? 0 : 1);
        beam3Smoother.add(s3.getState() ? 0 : 1);

        st1 = beam1Smoother.mode() == 1;
        st2 = beam2Smoother.mode() == 1;
        st3 = beam3Smoother.mode() == 1;
    }

    private void updateStoredCount() {
        updateNormalTransitions();
        updateS1Fallback();
    }

    enum StorageState {
        WAIT_FOR_ST3,
        DELAY_AFTER_ST3,
        WAIT_FOR_ST2,
        DELAY_AFTER_ST2,
        WAIT_FOR_ST1,
        DELAY_AFTER_ST1,
        FULL
    }

    StorageState storageState;
    public static double ST1_DELAY = 0.15, ST2_DELAY = 0.38, ST3_DELAY = 0.25; // 0.18, 0.32, 0 // TODO: KEEP TUNING

    private void updateNormalTransitions() {
        switch(storageState) {
            case WAIT_FOR_ST3:
                if (st3) {
                    timer.resetTimer();
                    storageState = StorageState.DELAY_AFTER_ST3;
                }
                break;
            case DELAY_AFTER_ST3:
                if(timer.getElapsedTimeSeconds() > ST3_DELAY){
                    storedCount = 1;
                    storageState = StorageState.WAIT_FOR_ST2;
                }
                break;
            case WAIT_FOR_ST2:
                if (st2 && !st1) {
                    timer.resetTimer();
                    storageState = StorageState.DELAY_AFTER_ST2;
                }
                break;
            case DELAY_AFTER_ST2:
                if(timer.getElapsedTimeSeconds() > ST2_DELAY){
                    storedCount = 2;
                    storageState = StorageState.WAIT_FOR_ST1;
                }
                break;
            case WAIT_FOR_ST1:
                if (st1) {
                    timer.resetTimer();
                    storageState = StorageState.DELAY_AFTER_ST1;
                }
                break;
            case DELAY_AFTER_ST1:
                if(timer.getElapsedTimeSeconds() > ST1_DELAY){
                    // storedCount = 3;
                    //  storageState = StorageState.FULL;
                }
                break;
            case FULL:
                break;
        }
    }

    // Fallback: if s1 (the last/full slot) stays active for a long time, assume the whole magazine
    // is full regardless of what the normal sequence count thinks.
    private void updateS1Fallback() {
        if (st1) {
            s1FullLoops++;
            if (s1FullLoops >= s1FullHoldLoops) {
                storedCount = 3;
                storageState = StorageState.FULL;
                s1HardFull = true;
            }
        } else {
            s1FullLoops = 0;
        }
    }
}