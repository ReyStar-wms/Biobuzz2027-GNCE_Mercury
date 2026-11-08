package org.firstinspires.ftc.teamcode.config.core.util;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.Artifact;

import java.util.LinkedList;
import java.util.Queue;
public class ArtifactDataSmoother {
    private final int windowSize;
    private final Queue<Artifact> window = new LinkedList<>();
    private Artifact lastStable = Artifact.NONE;

    private int greenCount = 0;
    private int purpleCount = 0;
    private int noneCount = 0;

    public ArtifactDataSmoother(int windowSize) {
        this.windowSize = Math.max(1, windowSize);
        greenCount = purpleCount = noneCount = 0;
    }

    public void addReading(boolean isThere, boolean isGreen) {
        boolean isPurple = isThere && !isGreen;
        Artifact current;

        // Handle conflicting signals
        if (isGreen) {
            current = Artifact.GREEN;
        } else if (isPurple) {
            current = Artifact.PURPLE;
        } else {
            current = Artifact.NONE;
        }

        // Remove oldest if window full
        if (window.size() >= windowSize) {
            Artifact removed = window.poll();
            decrementCount(removed);
        }

        // Add new reading
        window.offer(current);
        incrementCount(current);

        // Update stable color
        lastStable = computeStableColor();
    }

    private void incrementCount(Artifact artifact) {
        if (artifact == Artifact.GREEN) greenCount++;
        else if (artifact == Artifact.PURPLE) purpleCount++;
        else if (artifact == Artifact.NONE) noneCount++;
    }

    private void decrementCount(Artifact artifact) {
        if (artifact == Artifact.GREEN) greenCount--;
        else if (artifact == Artifact.PURPLE) purpleCount--;
        else if (artifact == Artifact.NONE) noneCount--;
    }

    private Artifact computeStableColor() {
        if (greenCount > purpleCount && greenCount >= 2)
            return Artifact.GREEN;
        if (purpleCount > greenCount && purpleCount >= 2)
            return Artifact.PURPLE;
        return Artifact.NONE;
    }

    public Artifact getStableColor() {
        return lastStable;
    }

    public void reset() {
        window.clear();
        greenCount = purpleCount = noneCount = 0;
        lastStable = Artifact.NONE;
    }
}
