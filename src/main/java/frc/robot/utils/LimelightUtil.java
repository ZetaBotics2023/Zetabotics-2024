package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * A utility class to use our Limelight to get estimated robot positions
 */
 public class LimelightUtil {

    // An instance of our Limelight's Network Table
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-zeta");

    /**
     * Returns an estimated bot position based off of
     * Limelight camera readings
     * @return The estimated robot position
     */
    public static VisionPose getBotpose() {

        // Retrieve the current estimated robot position from Network Tables
        double[] botpose = table.getEntry("botpose_wpi" + (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? "blue" : "red")).getDoubleArray(new double[6]);

        // Retrieve the current latency from Network Tables to coordinate rate of values
        double piplineLatency = table.getEntry("tl").getDouble(0);
        double capturePiplineLatency = table.getEntry("cl").getDouble(0);

        // Retrieve whether the Limelight sees a valid target
        boolean validTarget = (1 == table.getEntry("tv").getDouble(0));

        // Calculate the system time so we know when we read our current value
        double timeStamp = Timer.getFPGATimestamp() - (piplineLatency/1000.0) - (capturePiplineLatency/1000.0);

        // Create an estimated position object
        Pose2d position = new Pose2d(
            new Translation2d(botpose[0], botpose[1]),
            Rotation2d.fromDegrees(botpose[5]));
        SmartDashboard.putNumber("Lime Light angle,", Units.degreesToRadians(botpose[5]));

        // Return a new Vision Pose
        return new VisionPose(position, timeStamp, validTarget);
    }
}
