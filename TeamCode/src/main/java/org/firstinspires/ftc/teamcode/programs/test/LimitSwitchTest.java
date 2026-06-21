package org.firstinspires.ftc.teamcode.programs.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;

@Config
@TeleOp(name = "Limit Switch Test", group = "OpModes")
public class LimitSwitchTest extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    FtcDashboard dashboard;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        dashboard = FtcDashboard.getInstance();

        robot.initializeHardware(hardwareMap);
        robot.initialize();
    }

    @Override
    public void run() {
        if (robot.allHubs != null) {
            for (LynxModule hub : robot.allHubs) {
                hub.clearBulkCache();
            }
        }

        CommandScheduler.getInstance().run();

        telemetry.addData("Pedal -> Back", robot.backArtefacts.isPressed());
        telemetry.addData("Pedal -> Front", robot.frontArtefacts.isPressed());
        telemetry.addData("3 artefacts 1?", robot.are3Artefacts_1.isPressed());
        telemetry.addData("3 artefacts 2?", robot.are3Artefacts_2.isPressed());
        telemetry.addData("sorter artefact?", robot.sorterArtefact.isPressed());
        telemetry.update();
    }
}
