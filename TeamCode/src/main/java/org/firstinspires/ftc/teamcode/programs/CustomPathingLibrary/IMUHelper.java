package org.firstinspires.ftc.teamcode.programs.CustomPathingLibrary;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

/** Modern REV IMU helper: yaw in radians, resettable. */
public class IMUHelper {
    private IMU imu;
    private double yawOffset = 0.0;

    public void init(HardwareMap hw){
        imu = hw.get(IMU.class, "imu"); // name usually "imu"
        // Set your hub mounting orientation here:
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(params);
        imu.resetYaw();
        yawOffset = 0.0;
    }

    /** Heading in radians, CCW+, field frame relative (after any resets). */
    public double headingRad(){
        // IMU gives yaw in degrees; convert. Positive CCW by default with the above params.
        double yawDeg = imu.getRobotYawPitchRollAngles().getYaw( /*AngleUnit.DEGREES default*/ );
        double h = Math.toRadians(yawDeg) + yawOffset;
        // wrap to (-pi, pi]
        while (h <= -Math.PI) h += 2*Math.PI;
        while (h >   Math.PI) h -= 2*Math.PI;
        return h;
    }

    /** Zero current heading to 'desired' (e.g., 0 or Math.PI). */
    public void setHeadingRad(double desired){
        double cur = headingRad();
        yawOffset += (desired - cur);
        // normalize
        while (yawOffset <= -Math.PI) yawOffset += 2*Math.PI;
        while (yawOffset >   Math.PI) yawOffset -= 2*Math.PI;
    }
}
