package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.config.core.util.robothelper.Alliance;

import java.util.EnumMap;

public final class AutoPaths2 {
    // ====== POSES (BLUE ONLY) ======

    // POSE FOR RED
    // START_FRONT (new Pose(25.34814419, 134, Math.toRadians(-127.879678))),

    static final double X_OFFSET_FRONT = -7.414;
    static final double Y_OFFSET_FRONT = -1.617463638;

    static final double GATE_INTAKE_Y_OFFSET = 0.0725; // 0.1725 for pf
    // 5.78, 9.23
    // 5.45, 9.485
    // 133.95 6.866
    // 120.348, 132.2373, -142.08185
    // 34.125 135.9, -90

    // 133.1 6.4271

    // 6.40419

    // Current Pose: (26.71078354208486, 134.28253636247763, -89.53565892394403)


    static double SINGLE_X_OFFSET = -1.5;

    public enum PoseId {
        GATE_INTAKE_TRIPLE (new Pose(18.25+ X_OFFSET_FRONT, 60+Y_OFFSET_FRONT, Math.toRadians(152))), // 12.3 56.66 150.14
        GATE_INTAKE_SAFE_TRIPLE (new Pose(19+ X_OFFSET_FRONT, 55.75+Y_OFFSET_FRONT, Math.toRadians(160))),
        GATE_INTAKE_SAFE_TRIPLE_SAFE (new Pose(19+ X_OFFSET_FRONT, 57+Y_OFFSET_FRONT, Math.toRadians(180))),

        // SINGLE GATE CYCLE (Van_K)
        GATE_INTAKE_ONLY_ONE_1 (new Pose(10.5, 58.25, Math.toRadians(158))),
        GATE_INTAKE_ONLY_ONE_2 (new Pose(10.5, 58.25 + GATE_INTAKE_Y_OFFSET, Math.toRadians(158))),
        GATE_INTAKE_ONLY_ONE_3 (new Pose(10.5, 58.25 + GATE_INTAKE_Y_OFFSET*2, Math.toRadians(158))),
        GATE_INTAKE_ONLY_ONE_4 (new Pose(10.5, 58.25 + GATE_INTAKE_Y_OFFSET*3, Math.toRadians(158))),
        // Near-identical to GATE_INTAKE_ONLY_ONE (y - 1 so the robot actually moves) but rotatedzfy8
        // to the shoot heading, so it turns in place before driving straight back to shoot.
        GATE_INTAKE_ONLY_ONE_SAFE (new Pose(10.5, 55.15, Math.toRadians(195))),

        FRONT_SHOOT_AFTER_GATE (new Pose(63.268+ X_OFFSET_FRONT, 77.5655+Y_OFFSET_FRONT, Math.toRadians(215))),
        FRONT_SHOOT_AFTER_GATE_FINAL (new Pose(63.268+ X_OFFSET_FRONT, 77.5655+Y_OFFSET_FRONT, Math.toRadians(255))),
        FRONT_SHOOT_AFTER_GATE_FINAL_FINAL (new Pose(63.268+ X_OFFSET_FRONT, 77.5655+Y_OFFSET_FRONT, Math.toRadians(210))),

        FRONT_SHOOT_AFTER_GATE_NEW (new Pose(63.268+ X_OFFSET_FRONT, 77.5655+Y_OFFSET_FRONT, Math.toRadians(215))),
        FRONT_SHOOT_AFTER_GATE_FINAL_18 (new Pose(63.268+ X_OFFSET_FRONT, 77.5655+Y_OFFSET_FRONT, Math.toRadians(205))),

        FRONT_START(new Pose(19, 110.74, Math.toRadians(180))),
        MID_SPIKE_START(new Pose(61, 73.5, Math.toRadians(220))),
        MID_SPIKE_CONTROL(new Pose(60, 60)),
        MID_SPIKE_CONTROL_BACK(new Pose(60, 65)),
        MID_SPIKE_END(new Pose(18, 56, Math.toRadians(180))),
        CLOSE_SPIKE_START(new Pose(42.578, 83.5327, Math.toRadians(180))),
        CLOSE_SPIKE_END(new Pose(17, 84, Math.toRadians(180))),
        FINAL_SHOOT(new Pose(56, 101.5, Math.toRadians(200))),
        FINAL_SHOOT_BRUNSON(new Pose(44, 84, Math.toRadians(180))),

        BRUNSON_FINAL_PARK_POS(new Pose(41, 84, Math.toRadians(180))),

        SHOOT_BACK_1 (new Pose(57.3185, 20.1238, Math.toRadians(155))),
        TRIPLE_FAR_SPIKE_CONTROL (new Pose(57, 37)),
        TRIPLE_FAR_SPIKE_END(new Pose(10, 35.8, Math.toRadians(180))),

        INTERMEDIATE_FAR_INTAKE_POSE   (new Pose(54.5, 32.5)),
        FAR_SPIKE_END     (new Pose(17, 36, Math.toRadians(180)));


        private final Pose blue;

        PoseId(Pose blue) {
            this.blue = blue;
        }

        public Pose forAlliance(Alliance a) {
            return (a == Alliance.RED) ? blue.mirror() : blue;
        }
    }
    public enum PathId {
        SINGLE_CLOSE_START_TO_MID_SPIKE_START,
        BRUNSON_FINAL_PARK,
        SINGLE_MID_SPIKE_START_TO_MID_SPIKE_END,
        SINGLE_MID_SPIKE_END_TO_FRONT_SHOOT_AFTER_GATE,
        SINGLE_SHOOT_AFTER_GATE_TO_CLOSE_SPIKE_END,
        SINGLE_SHOOT_AFTER_GATE_TO_FAR_SPIKE_END,
        SINGLE_CLOSE_SPIKE_END_TO_CLOSE_FINAL_SHOOT,
        SINGLE_CLOSE_SPIKE_END_TO_CLOSE_FINAL_SHOOT_BRUNSON,
        SINGLE_FAR_SPIKE_END_TO_FRONT_SHOOT_FOR_CLOSE_PREP,
        SHOOT_TO_GATE_INTAKE_NORMAL,
        TRIPLE_GATE_INTAKE_TO_GATE_INTAKE_SAFE,
        SHOOT_BACK_1_TO_FAR_SPIKE_END,
        TRIPLE_GATE_SAFE_TO_GATE_INTAKE_SAFE_SAFE,
        GATE_INTAKE_SAFE_SAFE_TO_SHOOT,
        GATE_INTAKE_SAFE_SAFE_TO_SHOOT_FINAl,
        SHOOT_TO_GATE_INTAKE_ONLY_ONE_1,
        SHOOT_TO_GATE_INTAKE_ONLY_ONE_2,
        SHOOT_TO_GATE_INTAKE_ONLY_ONE_3,
        SHOOT_TO_GATE_INTAKE_ONLY_ONE_4,
        GATE_INTAKE_ONLY_ONE_TO_SHOOT_1,
        GATE_INTAKE_ONLY_ONE_TO_SHOOT_2,
        GATE_INTAKE_ONLY_ONE_TO_SHOOT_3,
        GATE_INTAKE_ONLY_ONE_TO_SHOOT_4,
        GATE_INTAKE_ONLY_ONE_TO_SHOOT_PARTNER,
    }

    // ====== INSTANCE STATE ======
    private final Follower f;
    private final Alliance a;

    private final EnumMap<PoseId, Pose> p     = new EnumMap<>(PoseId.class);
    private final EnumMap<PathId, PathChain> paths = new EnumMap<>(PathId.class);
    public AutoPaths2(Follower f, Alliance a) {
        this.f = f;
        this.a = a;

        // Build alliance-correct poses
        for (PoseId id : PoseId.values()) {
            p.put(id, id.forAlliance(a));
        }

        // Build all individual path chains
        buildPaths();
    }

    // ====== ACCESS ======
    public Pose getPose(PoseId id) {
        return p.get(id);
    }

    public PathChain getPath(PathId id) {
        return paths.get(id);
    }

    // ====== PATH DEFINITIONS ======
    private void buildPaths() {
        // CLOSE SINGLE AUTO
        put(PathId.SINGLE_CLOSE_START_TO_MID_SPIKE_START, PoseId.FRONT_START, PoseId.MID_SPIKE_START);


        paths.put(PathId.SINGLE_MID_SPIKE_START_TO_MID_SPIKE_END,
                f.pathBuilder()
                        .addPath(new BezierCurve(getPose(PoseId.MID_SPIKE_START), getPose(PoseId.MID_SPIKE_CONTROL), getPose(PoseId.MID_SPIKE_END)))
                        .setLinearHeadingInterpolation(getPose(PoseId.MID_SPIKE_START).getHeading(), getPose(PoseId.MID_SPIKE_END).getHeading())
                        .build()
        );

        paths.put(PathId.SINGLE_MID_SPIKE_END_TO_FRONT_SHOOT_AFTER_GATE,
                f.pathBuilder()
                        .addPath(new BezierLine(getPose(PoseId.MID_SPIKE_END), getPose(PoseId.MID_SPIKE_START)))
                        .setLinearHeadingInterpolation(getPose(PoseId.MID_SPIKE_END).getHeading(), getPose(PoseId.MID_SPIKE_START).getHeading())
                        .build()
        );

        put(PathId.SINGLE_CLOSE_SPIKE_END_TO_CLOSE_FINAL_SHOOT, PoseId.CLOSE_SPIKE_END, PoseId.FINAL_SHOOT);

        put(PathId.SINGLE_CLOSE_SPIKE_END_TO_CLOSE_FINAL_SHOOT_BRUNSON, PoseId.CLOSE_SPIKE_END, PoseId.FINAL_SHOOT_BRUNSON);

        put(PathId.BRUNSON_FINAL_PARK, PoseId.FINAL_SHOOT_BRUNSON, PoseId.BRUNSON_FINAL_PARK_POS);

        paths.put(PathId.SINGLE_SHOOT_AFTER_GATE_TO_CLOSE_SPIKE_END,
                f.pathBuilder()
                        .addPath(new BezierLine(getPose(PoseId.FRONT_SHOOT_AFTER_GATE_FINAL_18), getPose(PoseId.CLOSE_SPIKE_END)))
                        .setLinearHeadingInterpolation(getPose(PoseId.FRONT_SHOOT_AFTER_GATE_FINAL_18).getHeading(), getPose(PoseId.CLOSE_SPIKE_END).getHeading())
                        .build()
        );

        paths.put(PathId.SINGLE_FAR_SPIKE_END_TO_FRONT_SHOOT_FOR_CLOSE_PREP,
                f.pathBuilder()
                        .addPath(new BezierLine(getPose(PoseId.FAR_SPIKE_END), getPose(PoseId.FRONT_SHOOT_AFTER_GATE_FINAL_FINAL)))
                        .setLinearHeadingInterpolation(getPose(PoseId.FAR_SPIKE_END).getHeading(), getPose(PoseId.FRONT_SHOOT_AFTER_GATE_FINAL_FINAL).getHeading())
                        .build()
        );

        paths.put(PathId.SINGLE_SHOOT_AFTER_GATE_TO_FAR_SPIKE_END,
                f.pathBuilder()
                        .addPath(new BezierCurve(getPose(PoseId.FRONT_SHOOT_AFTER_GATE_FINAL), getPose(PoseId.INTERMEDIATE_FAR_INTAKE_POSE), getPose(PoseId.FAR_SPIKE_END)))
                        .setLinearHeadingInterpolation(getPose(PoseId.FRONT_SHOOT_AFTER_GATE_FINAL).getHeading(), getPose(PoseId.FAR_SPIKE_END).getHeading())
                        .build()
        );

        paths.put(PathId.SHOOT_BACK_1_TO_FAR_SPIKE_END,
                f.pathBuilder()
                        .addPath(new BezierCurve(getPose(PoseId.SHOOT_BACK_1), getPose(PoseId.TRIPLE_FAR_SPIKE_CONTROL), getPose(PoseId.TRIPLE_FAR_SPIKE_END)))
                        .setLinearHeadingInterpolation(getPose(PoseId.SHOOT_BACK_1).getHeading(), getPose(PoseId.TRIPLE_FAR_SPIKE_END).getHeading())
                        .build()
        );

        put(PathId.SHOOT_BACK_1_TO_FAR_SPIKE_END, PoseId.SHOOT_BACK_1, PoseId.TRIPLE_FAR_SPIKE_END);
        paths.put(PathId.SHOOT_TO_GATE_INTAKE_NORMAL,
                f.pathBuilder()
                        .addPath(new BezierLine(getPose(PoseId.FRONT_SHOOT_AFTER_GATE), getPose(PoseId.GATE_INTAKE_TRIPLE)))
                        .setHeadingInterpolation(HeadingInterpolator.piecewise(
                                HeadingInterpolator.PiecewiseNode.linear(0.0, 0.5, getPose(PoseId.FRONT_SHOOT_AFTER_GATE).getHeading(), getPose(PoseId.GATE_INTAKE_TRIPLE).getHeading()),
                                HeadingInterpolator.PiecewiseNode.linear(0.5, 1, getPose(PoseId.GATE_INTAKE_TRIPLE).getHeading(), getPose(PoseId.GATE_INTAKE_TRIPLE).getHeading())
                        ))
                        .build()
        );
        put(PathId.TRIPLE_GATE_INTAKE_TO_GATE_INTAKE_SAFE, PoseId.GATE_INTAKE_TRIPLE, PoseId.GATE_INTAKE_SAFE_TRIPLE);

        paths.put(PathId.TRIPLE_GATE_SAFE_TO_GATE_INTAKE_SAFE_SAFE,
                f.pathBuilder()
                        .addPath(new BezierLine(getPose(PoseId.GATE_INTAKE_SAFE_TRIPLE), getPose(PoseId.GATE_INTAKE_SAFE_TRIPLE_SAFE)))
                        .setHeadingInterpolation(HeadingInterpolator.piecewise(
                                HeadingInterpolator.PiecewiseNode.linear(0.0, 0.3, getPose(PoseId.GATE_INTAKE_SAFE_TRIPLE).getHeading(), getPose(PoseId.GATE_INTAKE_SAFE_TRIPLE_SAFE).getHeading()),
                                HeadingInterpolator.PiecewiseNode.linear(0.3, 1, getPose(PoseId.GATE_INTAKE_SAFE_TRIPLE).getHeading(), getPose(PoseId.GATE_INTAKE_SAFE_TRIPLE_SAFE).getHeading())
                        ))
                        .build()
        );

        paths.put(PathId.GATE_INTAKE_SAFE_SAFE_TO_SHOOT,
                f.pathBuilder()
                        .addPath(new BezierLine(getPose(PoseId.GATE_INTAKE_SAFE_TRIPLE_SAFE), getPose(PoseId.FRONT_SHOOT_AFTER_GATE)))
                        .setHeadingInterpolation(HeadingInterpolator.piecewise(
                                HeadingInterpolator.PiecewiseNode.linear(0.0, 0.3, getPose(PoseId.GATE_INTAKE_SAFE_TRIPLE_SAFE).getHeading(), getPose(PoseId.FRONT_SHOOT_AFTER_GATE).getHeading()),
                                HeadingInterpolator.PiecewiseNode.linear(0.3, 1, getPose(PoseId.FRONT_SHOOT_AFTER_GATE).getHeading(), getPose(PoseId.FRONT_SHOOT_AFTER_GATE).getHeading())
                        ))
                        .build()
        );

        paths.put(PathId.GATE_INTAKE_SAFE_SAFE_TO_SHOOT_FINAl,
                f.pathBuilder()
                        .addPath(new BezierLine(getPose(PoseId.GATE_INTAKE_SAFE_TRIPLE_SAFE), getPose(PoseId.FRONT_SHOOT_AFTER_GATE_FINAL)))
                        .setHeadingInterpolation(HeadingInterpolator.piecewise(
                                HeadingInterpolator.PiecewiseNode.linear(0.0, 0.3, getPose(PoseId.GATE_INTAKE_SAFE_TRIPLE_SAFE).getHeading(), getPose(PoseId.FRONT_SHOOT_AFTER_GATE_FINAL).getHeading()),
                                HeadingInterpolator.PiecewiseNode.linear(0.3, 1, getPose(PoseId.FRONT_SHOOT_AFTER_GATE_FINAL).getHeading(), getPose(PoseId.FRONT_SHOOT_AFTER_GATE_FINAL).getHeading())
                        ))
                        .build()
        );

        // ====== SINGLE GATE CYCLE (Van_K) ======
        // Shoot position -> single gate intake. Mirrors SHOOT_TO_GATE_INTAKE_NORMAL: reach the
        // gate heading by the halfway point, then hold it for the intake.
        paths.put(PathId.SHOOT_TO_GATE_INTAKE_ONLY_ONE_1,
                f.pathBuilder()
                        .addPath(new BezierLine(getPose(PoseId.MID_SPIKE_START), getPose(PoseId.GATE_INTAKE_ONLY_ONE_1)))
                        .setHeadingInterpolation(HeadingInterpolator.piecewise(
                                HeadingInterpolator.PiecewiseNode.linear(0.0, 0.5, getPose(PoseId.MID_SPIKE_START).getHeading(), getPose(PoseId.GATE_INTAKE_ONLY_ONE_1).getHeading()),
                                HeadingInterpolator.PiecewiseNode.linear(0.5, 1, getPose(PoseId.GATE_INTAKE_ONLY_ONE_1).getHeading(), getPose(PoseId.GATE_INTAKE_ONLY_ONE_1).getHeading())
                        ))
                        .build()
        );

        paths.put(PathId.SHOOT_TO_GATE_INTAKE_ONLY_ONE_2,
                f.pathBuilder()
                        .addPath(new BezierLine(getPose(PoseId.MID_SPIKE_START), getPose(PoseId.GATE_INTAKE_ONLY_ONE_2)))
                        .setHeadingInterpolation(HeadingInterpolator.piecewise(
                                HeadingInterpolator.PiecewiseNode.linear(0.0, 0.5, getPose(PoseId.MID_SPIKE_START).getHeading(), getPose(PoseId.GATE_INTAKE_ONLY_ONE_2).getHeading()),
                                HeadingInterpolator.PiecewiseNode.linear(0.5, 1, getPose(PoseId.GATE_INTAKE_ONLY_ONE_2).getHeading(), getPose(PoseId.GATE_INTAKE_ONLY_ONE_2).getHeading())
                        ))
                        .build()
        );

        paths.put(PathId.SHOOT_TO_GATE_INTAKE_ONLY_ONE_3,
                f.pathBuilder()
                        .addPath(new BezierLine(getPose(PoseId.FRONT_SHOOT_AFTER_GATE_NEW), getPose(PoseId.GATE_INTAKE_ONLY_ONE_3)))
                        .setHeadingInterpolation(HeadingInterpolator.piecewise(
                                HeadingInterpolator.PiecewiseNode.linear(0.0, 0.5, getPose(PoseId.FRONT_SHOOT_AFTER_GATE_NEW).getHeading(), getPose(PoseId.GATE_INTAKE_ONLY_ONE_3).getHeading()),
                                HeadingInterpolator.PiecewiseNode.linear(0.5, 1, getPose(PoseId.GATE_INTAKE_ONLY_ONE_3).getHeading(), getPose(PoseId.GATE_INTAKE_ONLY_ONE_3).getHeading())
                        ))
                        .build()
        );

        paths.put(PathId.SHOOT_TO_GATE_INTAKE_ONLY_ONE_4,
                f.pathBuilder()
                        .addPath(new BezierLine(getPose(PoseId.FRONT_SHOOT_AFTER_GATE_NEW), getPose(PoseId.GATE_INTAKE_ONLY_ONE_4)))
                        .setHeadingInterpolation(HeadingInterpolator.piecewise(
                                HeadingInterpolator.PiecewiseNode.linear(0.0, 0.5, getPose(PoseId.FRONT_SHOOT_AFTER_GATE_NEW).getHeading(), getPose(PoseId.GATE_INTAKE_ONLY_ONE_4).getHeading()),
                                HeadingInterpolator.PiecewiseNode.linear(0.5, 1, getPose(PoseId.GATE_INTAKE_ONLY_ONE_4).getHeading(), getPose(PoseId.GATE_INTAKE_ONLY_ONE_4).getHeading())
                        ))
                        .build()
        );

        // Single gate intake -> shoot, in two steps:
        //   1) Turn in place to the shoot heading (tiny y move so the follower actually runs).
        //   2) Drive straight back to the shoot pose at a constant heading (no strafing).
        paths.put(PathId.GATE_INTAKE_ONLY_ONE_TO_SHOOT_1,
                f.pathBuilder()
                        .addPath(new BezierLine(getPose(PoseId.GATE_INTAKE_ONLY_ONE_1), getPose(PoseId.GATE_INTAKE_ONLY_ONE_SAFE)))
                        .setLinearHeadingInterpolation(getPose(PoseId.GATE_INTAKE_ONLY_ONE_1).getHeading(), getPose(PoseId.GATE_INTAKE_ONLY_ONE_SAFE).getHeading())
                        .addPath(new BezierLine(getPose(PoseId.GATE_INTAKE_ONLY_ONE_SAFE), getPose(PoseId.MID_SPIKE_START)))
                        .setConstantHeadingInterpolation(getPose(PoseId.MID_SPIKE_START).getHeading())
                        .build()
        );

        paths.put(PathId.GATE_INTAKE_ONLY_ONE_TO_SHOOT_2,
                f.pathBuilder()
                        .addPath(new BezierLine(getPose(PoseId.GATE_INTAKE_ONLY_ONE_2), getPose(PoseId.GATE_INTAKE_ONLY_ONE_SAFE)))
                        .setLinearHeadingInterpolation(getPose(PoseId.GATE_INTAKE_ONLY_ONE_2).getHeading(), getPose(PoseId.GATE_INTAKE_ONLY_ONE_SAFE).getHeading())
                        .addPath(new BezierLine(getPose(PoseId.GATE_INTAKE_ONLY_ONE_SAFE), getPose(PoseId.FRONT_SHOOT_AFTER_GATE_NEW)))
                        .setConstantHeadingInterpolation(getPose(PoseId.FRONT_SHOOT_AFTER_GATE).getHeading())
                        .build()
        );

        paths.put(PathId.GATE_INTAKE_ONLY_ONE_TO_SHOOT_3,
                f.pathBuilder()
                        .addPath(new BezierLine(getPose(PoseId.GATE_INTAKE_ONLY_ONE_3), getPose(PoseId.GATE_INTAKE_ONLY_ONE_SAFE)))
                        .setLinearHeadingInterpolation(getPose(PoseId.GATE_INTAKE_ONLY_ONE_3).getHeading(), getPose(PoseId.GATE_INTAKE_ONLY_ONE_SAFE).getHeading())
                        .addPath(new BezierLine(getPose(PoseId.GATE_INTAKE_ONLY_ONE_SAFE), getPose(PoseId.FRONT_SHOOT_AFTER_GATE_FINAL_18)))
                        .setConstantHeadingInterpolation(getPose(PoseId.FRONT_SHOOT_AFTER_GATE_FINAL_18).getHeading())
                        .build()
        );

        paths.put(PathId.GATE_INTAKE_ONLY_ONE_TO_SHOOT_4,
                f.pathBuilder()
                        .addPath(new BezierLine(getPose(PoseId.GATE_INTAKE_ONLY_ONE_4), getPose(PoseId.GATE_INTAKE_ONLY_ONE_SAFE)))
                        .setLinearHeadingInterpolation(getPose(PoseId.GATE_INTAKE_ONLY_ONE_4).getHeading(), getPose(PoseId.GATE_INTAKE_ONLY_ONE_SAFE).getHeading())
                        .addPath(new BezierLine(getPose(PoseId.GATE_INTAKE_ONLY_ONE_SAFE), getPose(PoseId.FRONT_SHOOT_AFTER_GATE_NEW)))
                        .setConstantHeadingInterpolation(getPose(PoseId.FRONT_SHOOT_AFTER_GATE).getHeading())
                        .build()
        );

        paths.put(PathId.GATE_INTAKE_ONLY_ONE_TO_SHOOT_PARTNER,
                f.pathBuilder()
                        .addPath(new BezierLine(getPose(PoseId.GATE_INTAKE_ONLY_ONE_1), getPose(PoseId.GATE_INTAKE_ONLY_ONE_SAFE)))
                        .setLinearHeadingInterpolation(getPose(PoseId.GATE_INTAKE_ONLY_ONE_1).getHeading(), getPose(PoseId.GATE_INTAKE_ONLY_ONE_SAFE).getHeading())
                        .addPath(new BezierLine(getPose(PoseId.GATE_INTAKE_ONLY_ONE_SAFE), getPose(PoseId.FRONT_SHOOT_AFTER_GATE_FINAL)))
                        .setConstantHeadingInterpolation(getPose(PoseId.FRONT_SHOOT_AFTER_GATE_FINAL).getHeading())
                        .build()
        );
    }
    // 33.18, 134.21
    private void put(PathId id, PoseId from, PoseId to) {
        paths.put(id,
                line(from, to).build()
        );
    }
    private PathBuilder line(PoseId from, PoseId to){
        Pose s = p.get(from);
        Pose e = p.get(to);
        return f.pathBuilder()
                .addPath(new BezierLine(s, e))
                .setLinearHeadingInterpolation(
                        s.getHeading(),
                        e.getHeading()
                );
    }
}