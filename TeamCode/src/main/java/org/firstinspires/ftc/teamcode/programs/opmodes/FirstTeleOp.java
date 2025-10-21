package org.firstinspires.ftc.teamcode.programs.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary.MecanumDrive;

@TeleOp(name = "FirstTeleOp", group= "Linear Opmode")
public class FirstTeleOp extends LinearOpMode {

    GamepadEx driver;
    MecanumDrive body;
    private ElapsedTime runtime = new ElapsedTime();
    private double serv0 = 0;

    @Override
    public void runOpMode() {

        driver = new GamepadEx(gamepad1);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        body = new MecanumDrive(hardwareMap);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            driver.readButtons();
            body.servo.setPosition(serv0);
            if (driver.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                body.Slow_Motion(driver, telemetry);
            }
            else{
                body.teleop(driver, telemetry);
            }

            if (driver.wasJustPressed(GamepadKeys.Button.X))
            {
                serv0+=0.01;
            }
            if (driver.wasJustPressed(GamepadKeys.Button.Y))
            {
                serv0-=0.01;
            }
            double pos = body.servo.getPosition();

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Pos: ", pos);
            telemetry.update();
        }
    }
}
