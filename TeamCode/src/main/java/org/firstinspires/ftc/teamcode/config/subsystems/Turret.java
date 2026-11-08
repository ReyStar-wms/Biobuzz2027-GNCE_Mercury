package org.firstinspires.ftc.teamcode.config.subsystems;

import androidx.core.math.MathUtils;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.core.SubsysCore;
import org.firstinspires.ftc.teamcode.config.hardware.CachedServo;

@Configurable
public class Turret extends SubsysCore {
    CachedServo s1, s2;
    AbsoluteAnalogEncoder e1, e2;
    Pose motifPose = new Pose(72, 150);
    public static double MANUAL_OFFSET = 0;
    public static double ENCODER_TURRET_OFFSET = -163;
    final double GEAR_RATIO = (double) 99 / 24 * 20 / 96;
    public static double SERVO_RANGE = 348;
    public static double TURRET_OFFSET = -180;
    public static double TURRET_SECOND_OFFSET = 0;
    public static double BOTH_SERVO_OFFSET = 0.01;
    public static double SECOND_SERVO_OFFSET = 0.014;
    public static double POSITION_TOLERANCE = 10;
    boolean alwaysAtTarget;

    double previousServoAngle, currentServoAngle;
    double previousPower;
    public static int wrapCount;
    double err;
    private Pose goalPose, robotPose;

    public double servoToTurretDegrees(double deg){
        return deg/GEAR_RATIO;
    }
    public double turretToServoDegrees(double deg){
        return deg*GEAR_RATIO;
    }

    public enum Target {
        MOTIF,
        GOAL,
        DISABLE
    }

    public static boolean testingOnly = false;

    Target target;
    public static double ENCODER_OFFSET = 0; // Matches servo position 0 to encoder position 0

    public void setTarget(Target target) {
        this.target = target;
    }

    public Turret(int initialWrap){ // %TODO: isRed should not be here. It should be able to change during initialization
        e1 = new AbsoluteAnalogEncoder(h, "te1", 3.3, AngleUnit.DEGREES);
        ServoImplEx rawS1 = h.get(ServoImplEx.class, "ts1");
        ServoImplEx rawS2 = h.get(ServoImplEx.class, "ts2");
        rawS1.setDirection(Servo.Direction.REVERSE);
        rawS2.setDirection(Servo.Direction.REVERSE);
        rawS1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rawS2.setPwmRange(new PwmControl.PwmRange(500, 2500));
        // Cache writes: the turret holds its PWM, so only push a new position when it actually moves.
        s1 = new CachedServo(rawS1);
        s2 = new CachedServo(rawS2);

        wrapCount = initialWrap;
        previousServoAngle = e1.getCurrentPosition();
        previousPower = 0;
        testingOnly = false;
        alwaysAtTarget = true; // Todo
        setTarget(Target.GOAL);
    }

    public void setAlwaysAtTarget(boolean alwaysAtTarget){
        this.alwaysAtTarget = alwaysAtTarget;
    }
    double getCurrentTurretAngle(){
        return (servoToTurretDegrees(wrapCount*360+currentServoAngle+ENCODER_TURRET_OFFSET) + 180) % 360 - 180;
    }

    double calculateTargetPose(Pose cur, Pose tar){
        double turretTarget = Math.toDegrees(Math.atan2(tar.getY() - cur.getY(), tar.getX() - cur.getX()) - cur.getHeading() - Math.PI);
        turretTarget = (turretTarget + 540)%360-180;
        return -turretTarget;
    }

    public static double targetTurret = 0;

    public void input(Pose robotPose, Pose goalPose){
        this.robotPose = robotPose;
        this.goalPose = goalPose;
    }

    @Override
    public void periodic() {
        currentServoAngle = e1.getCurrentPosition();
        double deltaAngle = currentServoAngle - previousServoAngle;
        if(deltaAngle < -180) wrapCount++;
        else if (deltaAngle > 180) wrapCount--;

        Pose targetPose = null;
        switch (target){
            case GOAL:
                targetPose = goalPose;
                break;
            case MOTIF:
                targetPose = motifPose;
                break;
            case DISABLE:
                targetTurret = 0;
                break;
        }

        if(!testingOnly && target != Target.DISABLE) targetTurret = calculateTargetPose(robotPose, targetPose);

        double actualTargetTurret = targetTurret;
        actualTargetTurret += MANUAL_OFFSET;
        actualTargetTurret = MathUtils.clamp(actualTargetTurret, -180, 180); // -180 ~ 180
        err = actualTargetTurret - getCurrentTurretAngle();

        double sp = turretToServoDegrees((actualTargetTurret + TURRET_OFFSET + 360)%360 + TURRET_SECOND_OFFSET)/SERVO_RANGE + BOTH_SERVO_OFFSET; // Needs 0 ~ 360

        s1.setPosition(sp);
        s2.setPosition(sp+SECOND_SERVO_OFFSET);

        if(telemetryEnabled){
            t.addData("Turret Wrap Count", wrapCount);
            t.addData("Turret Servo Target Position", sp);
            t.addData("Turret Target Angle", actualTargetTurret);
            t.addData("Turret Current Angle", getCurrentTurretAngle());
            t.addData("Turret Servo Angle", currentServoAngle);
            t.addData("Turret At Target?", reachedTarget());
        }
        previousServoAngle = currentServoAngle;
    }

    public boolean reachedTarget(){ return alwaysAtTarget || (((getCurrentTurretAngle() - (targetTurret + MANUAL_OFFSET) + 180) % 360 + 360) % 360 - 180 < POSITION_TOLERANCE); }

    public int getWrapCount() {
        return wrapCount;
    }
}
