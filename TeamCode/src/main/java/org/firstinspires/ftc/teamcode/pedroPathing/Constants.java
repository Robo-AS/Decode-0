package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.6)
            .forwardZeroPowerAcceleration(-28.213609111043983)
            .lateralZeroPowerAcceleration(-62.00894187182901)
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.075, 0, 0.01, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(0.95, 0, 0.003, 0.007))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.006,0.0,0.00001,0.0,0.0))
            .centripetalScaling(0.00035);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(76.42478690560408)
            .yVelocity(58.62435648760457)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(14.7)
            .strafePodX(13.7)
            .distanceUnit(DistanceUnit.CM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    /**
     These are the PathConstraints in order:
     tValueConstraint, velocityConstraint, translationalConstraint, headingConstraint, timeoutConstraint,
     brakingStrength, BEZIER_CURVE_SEARCH_LIMIT, brakingStart
     The BEZIER_CURVE_SEARCH_LIMIT should typically be left at 10 and shouldn't be changed.
     */
    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.009,
            50,
            1,
            10,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}