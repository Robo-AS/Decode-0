package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

public class Limelight extends SubsystemBase {
    private Limelight3A limelight;
    public double purpleTx = 0.0;
    public double purpleTy = 0.0;
    public double greenTx  = 0.0;
    public double greenTy  = 0.0;

    public List<Artifact> artifactList = new ArrayList<>();

    public void useAprilTagPipeline() {
        limelight.pipelineSwitch(1);
    }

    public void useArtifactPipeline() {
        limelight.pipelineSwitch(0);
    }

    public void initializeHardware(final HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public int getAprilTagID() {
        LLResult result = limelight.getLatestResult();
        if (result != null && !result.getFiducialResults().isEmpty()) {
            return result.getFiducialResults().get(0).getFiducialId();
        }
        return -1;
    }

    public void initialize(){
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void loop() {
        LLResult result = limelight.getLatestResult();
        if (result == null) return;

        artifactList.clear();
        purpleTx = 0.0;
        purpleTy = 0.0;
        greenTx = 0.0;
        greenTy = 0.0;

        double[] pythonOutputs = result.getPythonOutput();
        if (pythonOutputs != null && pythonOutputs.length >= 4) {
            purpleTx = pythonOutputs[0];
            purpleTy = pythonOutputs[1];
            greenTx = pythonOutputs[2];
            greenTy = pythonOutputs[3];
        }

        if (pythonOutputs != null && pythonOutputs.length >= 8) {
            // artifact : color tx ty width height
            int totalArtifacts = pythonOutputs.length / 5;
            for (int i = 0; i < totalArtifacts; i++) {
                int idx = i * 5;
                int type = (int) pythonOutputs[idx];
                double cx = pythonOutputs[idx + 1];
                double cy = pythonOutputs[idx + 2];
                double w = pythonOutputs[idx + 3];
                double h = pythonOutputs[idx + 4];

                Artifact artifact = new Artifact(type == 0 ? "purple" : "green", cx, cy, w, h);
                artifactList.add(artifact);
            }
        }
    }

    public boolean hasTargets() {
        return !(purpleTx == 0 && purpleTy == 0 && greenTx == 0 && greenTy == 0);
    }
    public static class Artifact {
        public final String type;
        public final double cx, cy, width, height;

        public Artifact(String type, double cx, double cy, double width, double height) {
            this.type = type;
            this.cx = cx;
            this.cy = cy;
            this.width = width;
            this.height = height;
        }
    }
}
