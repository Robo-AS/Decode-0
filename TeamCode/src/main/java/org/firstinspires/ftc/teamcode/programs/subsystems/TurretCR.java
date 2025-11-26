package org.firstinspires.ftc.teamcode.programs.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.programs.utils.RTPAxon;
import org.firstinspires.ftc.teamcode.programs.utils.Robot;
import java.util.HashMap;
import java.util.Map;

public class TurretCR extends SubsystemBase {

    private final Robot robot = Robot.getInstance();
    private final Limelight3A limelight;
    private RTPAxon axon;

    public static double targetAngle = 0.0;
    public static double targetRotation = 0.0;
    public static double kP = 0.006;
    public static double kI = 0.001;
    public static double kD = 0.000002;
    public static double MAX_ANGLE = 150.0;
    private boolean poseSynced = false;
    private int targetID = 20;

    public static class TagPose {
        public final double x, y, z;
        TagPose(double x, double y, double z){ this.x=x; this.y=y; this.z=z; }
    }

    //for pinpoint and mt2 later
    private static final Map<Integer, TagPose> FIELD_TAGS_LL = new HashMap<>();
    static {
        FIELD_TAGS_LL.put(20, new TagPose(-1.482, -1.413, 0.749));
        FIELD_TAGS_LL.put(24, new TagPose(-1.482,  1.413, 0.749));
    }

    public TurretCR() {
        this.limelight = robot.limelight;
        this.axon = robot.axon;
    }

    public void initialize(){
        limelight.start();
        limelight.pipelineSwitch(1);
        limelight.setPollRateHz(100);

        axon.initialize(); //get encoderZero
    }

    private void updateHeadings(int targetID) {
        LLResult ll = limelight.getLatestResult();
        boolean seesTargetID = false;

        for(LLResultTypes.FiducialResult apriltag : ll.getFiducialResults()){
            if(apriltag.getFiducialId() == targetID){
                seesTargetID = true;
                break;
            }
        }

        boolean useLL = ll != null && ll.isValid() && ll.getBotpose_MT2() != null && FIELD_TAGS_LL.containsKey(targetID) && seesTargetID;

        if (useLL) {
            targetAngle = axon.getCurrentAngle() + ll.getTx();

            //correct pinpoint with limelight data so it has as little error as possible
            Pose2D poseFromLL = new Pose2D(
                    DistanceUnit.METER,
                    ll.getBotpose_MT2().getPosition().x,
                    ll.getBotpose_MT2().getPosition().y,
                    AngleUnit.RADIANS,
                    ll.getBotpose_MT2().getOrientation().getYaw()
            );

            poseSynced = true;
        }
        else {
            //home turret if it loses sight of target for simplicity
            targetAngle = 0;
        }
    }

    public void loop(int targetID) {
        updateHeadings(targetID);

        targetRotation =  targetAngle; //current angle the turret is at + what it sees from ll

        double clampedTarget = Math.max(-MAX_ANGLE, Math.min(MAX_ANGLE, targetRotation));

        axon.updatePIDCoeffs(kP, kI, kD); //set pid coefficients for rtp axon

        axon.setTargetRotation(clampedTarget); // update targetRotation in rtp axon as well

        axon.update(); // pid logic in rtp axon
    }

    public void loopAuto(double target){
        targetRotation = target;

        double clampedTarget = Math.max(-MAX_ANGLE, Math.min(MAX_ANGLE, targetRotation));

        axon.updatePIDCoeffs(kP, kI, kD); //set pid coefficients for rtp axon

        axon.setTargetRotation(clampedTarget); // update targetRotation in rtp axon as well

        axon.update();
    }
}