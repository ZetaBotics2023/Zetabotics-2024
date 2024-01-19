package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;

 public class LimelightUtil {
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-zeta");

    public static VisionPose getBotpose(Optional<Alliance> alliance) {

        double[] botpose = table.getEntry("botpose_wpi" + (alliance.isPresent() ? (alliance.get() == Alliance.Blue ? "blue" : "red") : "blue")).getDoubleArray(new double[6]);
        double piplineLatency = table.getEntry("tl").getDouble(0);
        double capturePiplineLatency = table.getEntry("cl").getDouble(0);
        boolean validTarget = (1 == table.getEntry("tv").getDouble(0));
        // From docs Timer.getFPGATimestamp() - (tl/1000.0) - (cl/1000.0)
        // I feel like it should be multipication to get ms up to seconds but the docs show this so that is what we are starting with
        double timeStamp = Timer.getFPGATimestamp() - (piplineLatency/1000.0) - (capturePiplineLatency/1000.0);
        Pose3d position = new Pose3d(
            new Translation3d(botpose[0], botpose[1], botpose[2]),
            new Rotation3d(Units.degreesToRadians(botpose[3]), Units.degreesToRadians(botpose[4]),
                    Units.degreesToRadians(botpose[5])));

        /* 
        SmartDashboard.putNumberArray("Botpose Data", botpose);
        SmartDashboard.putNumberArray("Tag ID", table.getEntry("tid").getDoubleArray(new double[6]));
        SmartDashboard.putNumber("Botpose X", position.getX());
        SmartDashboard.putNumber("Botpose Y", position.getY());
        SmartDashboard.putNumber("Botpose Z", position.getZ());
        SmartDashboard.putNumber("Botpose Rotation", position.getRotation().getAngle());
        SmartDashboard.putNumber("Time Stamp", timeStamp);
        SmartDashboard.putBoolean("Valid Target", validTarget);
        */

        return new VisionPose(position, timeStamp, validTarget);
    }
}
