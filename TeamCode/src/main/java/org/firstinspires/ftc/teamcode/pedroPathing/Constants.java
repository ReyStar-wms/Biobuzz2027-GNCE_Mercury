package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.FusionLocalizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.hardware.PinpointVisionLocalizer;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.52)
            .forwardZeroPowerAcceleration(-47.814236929448604)
            .lateralZeroPowerAcceleration(-98.514583160192)

            //.translationalPIDFCoefficients(new PIDFCoefficients(0.34, 0, 0.024, 0.032))

            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.05, 0.03))

            //.drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.00024, 0.6, 0.06))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.3, 0.06510191780288611, 0.0018851224424528885))
            ;


    public static PathConstraints pathConstraints = new PathConstraints(0.97, 5, 1, 0.98);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(76.05330147330217)
            .yVelocity(57.21404020054134)
            .useBrakeModeInTeleOp(true);
    /*
    123.8 43.24 2.478
    121.81 48.767 -2.4555
     */

    public static PinpointConstants localizerConstants = new PinpointConstants() // NOTE: These values are accurate as of 11/15/2025 except for directions
            .forwardPodY(-2.46063)
            .strafePodX(-3.562992)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pp")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static MyFusionLocalizer fusionLocalizer;

//    MT2 PedroConv: (136.17774820324007, 7.820190522788536, 89.99969337043095, class com.pedropathing.geometry.PedroCoordinates)
//    Current Pose: (136.20049434508107, 7.5104759068906315, 89.99969072377472, class com.pedropathing.geometry.PedroCoordinates)
//    Current Pinpoint Pose: (134.12499615526576, 6.87205397237943, 89.99983857993809, class com.pedropathing.geometry.PedroCoordinates)
//
//    MT2 PedroConv: (65.41760734989974, 73.69693314225806, 45.00433946114446, class com.pedropathing.geometry.PedroCoordinates)
//    Current Pose: (65.3742591999495, 73.66887588668969, 45.00436689736027, class com.pedropathing.geometry.PedroCoordinates)
//    Current Pinpoint Pose: (64.80142908772146, 71.9588104007751, 45.00449893180754, class com.pedropathing.geometry.PedroCoordinates)
//
//
//    MT2 PedroConv: (64.92358647115239, 16.113733320211843, 91.04613209035386, class com.pedropathing.geometry.PedroCoordinates)
//    Current Pose: (64.9348771250549, 16.088563845167172, 91.04613884217038, class com.pedropathing.geometry.PedroCoordinates)
//    Current Pinpoint Pose: (62.139969472810044, 14.963965228223426, 91.04629186267574, class com.pedropathing.geometry.PedroCoordinates)
//
//    MT2 PedroConv: (8.502692211827792, 7.635838639285694, 89.69870085057543, class com.pedropathing.geometry.PedroCoordinates)
//    Current Pose: (8.506262528746658, 7.6890869685215355, 89.69869125447357, class com.pedropathing.geometry.PedroCoordinates)
//    Current Pinpoint Pose: (6.375563013272022, 7.525619747131828, 89.69885263378313, class com.pedropathing.geometry.PedroCoordinates)


    public static Follower createFollower(HardwareMap hardwareMap) {
        fusionLocalizer = new MyFusionLocalizer(
                new PinpointLocalizer(hardwareMap, localizerConstants),
                new Pose(0.25, 0.25),
                new Pose(0.0005, 0.0005),
                new Pose(0.2, 0.2),
                50
        );

        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .setLocalizer(fusionLocalizer)
                .build();
    }
}
