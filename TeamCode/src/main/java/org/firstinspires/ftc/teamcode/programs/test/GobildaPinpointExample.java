package org.firstinspires.ftc.teamcode.programs.test;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import java.util.Locale;

@TeleOp(name="goBILDA Pinpoint Example", group="Linear OpMode")
public class GobildaPinpointExample extends LinearOpMode {

    GoBildaPinpointDriver odo;
    double oldTime = 0;

    @Override
    public void runOpMode() {

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");

        odo.setOffsets(131.16, -6.717, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized and Reset");
        telemetry.update();

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            odo.update();

            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));

            String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));

            telemetry.addData("Position", data);
            telemetry.addData("Velocity", velocity);
            telemetry.addData("Status", odo.getDeviceStatus());
            telemetry.addData("Pinpoint Frequency", odo.getFrequency());
            telemetry.addData("REV Hub Frequency: ", frequency);
            telemetry.update();
        }
    }
}