package org.firstinspires.ftc.teamcode.programs.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.programs.subsystems.Camera;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "Camera Detection Test", group = "Test")
public class CameraDetectionTest extends CommandOpMode {

    private final Robot robot = Robot.getInstance();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private OpenCvWebcam webcam;
    private Camera pipeline;  // your class

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        CommandScheduler.getInstance().reset();

        robot.initializeHardware(hardwareMap);
        robot.initialize();

        //-------------------------
        // CREATE CAMERA INSTANCE
        //-------------------------
        int cameraMonitorViewId =
                hardwareMap.appContext.getResources().getIdentifier(
                        "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class,
                        "webcam"), cameraMonitorViewId);

        pipeline = new Camera();
        webcam.setPipeline(pipeline);

        //-------------------------
        // START THE CAMERA ASYNC
        //-------------------------
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                dashboard.startCameraStream(webcam, 30);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        telemetry.addLine("Camera Running...");
        telemetry.update();
    }
}
