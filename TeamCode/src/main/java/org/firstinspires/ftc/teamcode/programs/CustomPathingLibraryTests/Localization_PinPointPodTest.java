//package org.firstinspires.ftc.teamcode.programs.CustomPathingLibraryTests;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary.GoBildaPinpointLocalizer;
//import org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary.Pose2d;
//
//@TeleOp(name="Pinpoint Pod Direction Test", group="Tests")
//public class PinpointPodTest extends LinearOpMode {
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // Init Pinpoint Localizer
//        GoBildaPinpointLocalizer odo = new GoBildaPinpointLocalizer("pinpoint")
//                .setEncoderResolutionPreset(
//                        GoBildaPinpointLocalizer.GoBildaOdometryPods.goBILDA_4_BAR_POD
//                )
//                .setPodOffsetsMM(-84, -168) // <-- your measured pod geometry in mm
//                .setEncoderDirections(
//                        GoBildaPinpointLocalizer.EncoderDirection.FORWARD,
//                        GoBildaPinpointLocalizer.EncoderDirection.FORWARD
//                );
//
//        odo.init(hardwareMap);
//        odo.resetPosAndIMU(); // zero at start
//        odo.setPose(new Pose2d(0,0,0)); // field start pose
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            odo.update(0.02); // assume ~20ms loop
//
//            Pose2d p = odo.getPose();
//            telemetry.addData("X (forward)", "%.2f in", p.x);
//            telemetry.addData("Y (left)", "%.2f in", p.y);
//            telemetry.addData("Heading (deg)", "%.1f°", Math.toDegrees(p.heading));
//            telemetry.update();
//
//            sleep(20);
//        }
//    }
//}