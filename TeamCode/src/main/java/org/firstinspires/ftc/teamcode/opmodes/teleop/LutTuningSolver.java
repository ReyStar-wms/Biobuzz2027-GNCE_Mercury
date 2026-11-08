package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * LutTuningSolver
 *
 * Offline tuning aid (no hardware needed). For each shot distance it solves the projectile
 * equations for the LOWEST launch angle that still reaches goal height, clears the front lip,
 * and stays under the flywheel's max RPM, then prints the RPM that achieves it. The flattest
 * legal shot is the most tolerant to distance error, so these are the values we seed the
 * velocity LUT with.
 *
 * Physics: for launch speed v at angle th, trajectory height at horizontal range x is
 *     y(x) = x*tan(th) - g*x^2 / (2 v^2 cos^2(th))
 * Setting y(x) = dh (target rise above the launcher) and solving for v gives
 *     v^2 = g*x^2 / ( 2 cos^2(th) ( x*tan(th) - dh ) )      [valid when x*tan(th) > dh]
 * and RPM = v / EXIT_VEL_M_PER_RPM.
 *
 * Press PLAY and read the Driver Station / dashboard telemetry.
 */
@TeleOp(group = "Test", name = "LUT Tuning Solver")
public class LutTuningSolver extends LinearOpMode {

    // ---- Field / mechanism geometry (same constants as ShotPlanner) ----
    static final double POSE_UNITS_TO_METERS     = 0.0254;   // inches -> meters
    static final double EXIT_VEL_M_PER_RPM       = 0.00365;  // m/s of exit velocity per RPM
    static final double GOAL_HEIGHT_M            = 1.2575;   // center of goal opening, above floor
    static final double GOAL_LIP_HEIGHT_M        = 1.0;      // front lip the shot must clear
    static final double GOAL_DEPTH_M             = 1 * 0.0254;
    static final double LAUNCHER_HEIGHT_ACTUAL_M = 0.38;     // exit height of the launcher
    static final double GRAVITY                  = 9.80665;
    static final double MAX_RPM                  = 2800.0;

    // Distance sweep (inches) and the angle search window (deg from horizontal).
    static final double DIST_MIN_IN  = 30;
    static final double DIST_MAX_IN  = 150;
    static final double DIST_STEP_IN = 6;
    static final double SEARCH_MIN_DEG  = 25.0;
    static final double SEARCH_MAX_DEG  = 75.0;
    static final double SEARCH_STEP_DEG = 0.1;

    private double trajHeight(double th, double x, double v2) {
        double cos = Math.cos(th);
        return x * Math.tan(th) - (GRAVITY * x * x) / (2.0 * v2 * cos * cos);
    }

    @Override
    public void runOpMode() {
        double dh = GOAL_HEIGHT_M - LAUNCHER_HEIGHT_ACTUAL_M;
        double inPerM = 1.0 / POSE_UNITS_TO_METERS;

        // Build the whole table once (this is just math, no hardware).
        StringBuilder table = new StringBuilder();
        table.append(String.format("%-9s | %-11s | %-9s | %-6s | %-7s\n",
                "Dist(in)", "MinAng(deg)", "Hood(deg)", "RPM", "v(m/s)"));
        table.append("----------+-------------+-----------+--------+--------\n");

        for (double distIn = DIST_MIN_IN; distIn <= DIST_MAX_IN; distIn += DIST_STEP_IN) {
            double x = distIn * POSE_UNITS_TO_METERS;
            double xLip = x - GOAL_DEPTH_M;

            double bestDeg = Double.NaN, bestRpm = Double.NaN, bestV = Double.NaN;

            // Walk from the flattest angle upward; the first one that is physically reachable,
            // clears the lip, and fits the RPM budget is the lowest-angle optimum.
            for (double deg = SEARCH_MIN_DEG; deg <= SEARCH_MAX_DEG; deg += SEARCH_STEP_DEG) {
                double th = Math.toRadians(deg);
                double rise = x * Math.tan(th) - dh;
                if (rise <= 1e-6) continue;                  // too flat to reach goal height here

                double cos = Math.cos(th);
                double v2 = (GRAVITY * x * x) / (2.0 * cos * cos * rise);
                double v = Math.sqrt(v2);
                double rpm = v / EXIT_VEL_M_PER_RPM;

                if (rpm > MAX_RPM) continue;                 // over the flywheel budget at this angle

                if (xLip > 0) {                              // front-lip clearance
                    double hLip = trajHeight(th, xLip, v2) + LAUNCHER_HEIGHT_ACTUAL_M;
                    if (hLip < GOAL_LIP_HEIGHT_M) continue;
                }

                bestDeg = deg; bestRpm = rpm; bestV = v;
                break;                                       // lowest feasible angle found
            }

            if (Double.isNaN(bestDeg)) {
                table.append(String.format("%-9.0f | %-11s | %-9s | %-6s | %-7s\n",
                        distIn, "--", "--", "N/A", "--"));
            } else {
                table.append(String.format("%-9.0f | %-11.1f | %-9.1f | %-6.0f | %-7.2f\n",
                        distIn, bestDeg, 90.0 - bestDeg, Math.round(bestRpm / 5.0) * 5.0, bestV));
            }
        }

        telemetry.setMsTransmissionInterval(50);

        // Show it before and after PLAY so you can read it either way.
        while (opModeInInit() && !isStopRequested()) {
            emit(dh, inPerM, table.toString());
            sleep(100);
        }
        waitForStart();
        while (opModeIsActive()) {
            emit(dh, inPerM, table.toString());
            sleep(100);
        }
    }

    private void emit(double dh, double inPerM, String table) {
        telemetry.addLine("=============== LUT TUNING SOLVER ===============");
        telemetry.addData("Goal height",  "%.2f in (%.4f m)", GOAL_HEIGHT_M * inPerM, GOAL_HEIGHT_M);
        telemetry.addData("Front lip",    "%.2f in (%.4f m)", GOAL_LIP_HEIGHT_M * inPerM, GOAL_LIP_HEIGHT_M);
        telemetry.addData("Launcher hgt", "%.2f in (%.4f m)", LAUNCHER_HEIGHT_ACTUAL_M * inPerM, LAUNCHER_HEIGHT_ACTUAL_M);
        telemetry.addData("Net rise",     "%.2f in (%.4f m)", dh * inPerM, dh);
        telemetry.addData("Max RPM",      "%.0f", MAX_RPM);
        telemetry.addLine("-------------------------------------------------");
        telemetry.addLine(table);
        telemetry.addLine("Lowest angle = flattest legal shot (most distance-");
        telemetry.addLine("tolerant) within lip + max-RPM constraints. RPM ~5.");
        telemetry.update();
    }
}
