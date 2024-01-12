package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;

/**
 * Command for teleop driving where translation is field oriented and rotation
 * velocity is controlled by the driver.
 * 
 * Translation is specified on the field-relative coordinate system. The Y-axis
 * runs parallel to the alliance wall, left
 * is positive. The X-axis runs down field toward the opposing alliance wall,
 * away from the alliance wall is positive.
 */
public class FieldOrientedDriveCommand extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;
  /**
   * Constructor
   * 
   * @param m_driveSubsystem  drivetrain
   * @param robotAngleSupplier   supplier for the current angle of the robot
   * @param translationXSupplier supplier for translation X component, in meters
   *                             per second
   * @param translationYSupplier supplier for translation Y component, in meters
   *                             per second
   * @param rotationSupplier     supplier for rotation component, in radians per
   *                             second
   */
  public FieldOrientedDriveCommand(
      DriveSubsystem m_driveSubsystem,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier) {
    this.m_driveSubsystem = m_driveSubsystem;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;

    addRequirements(m_driveSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
      m_driveSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            this.translationXSupplier.getAsDouble() * SwerveDriveConstants.kMaxSpeedMetersPerSecond,
            this.translationYSupplier.getAsDouble() * SwerveDriveConstants.kMaxSpeedMetersPerSecond,
            this.rotationSupplier.getAsDouble() * SwerveDriveConstants.kMaxRotationAnglePerSecond, 
            this.m_driveSubsystem.getPoseEstimatorSubsystem().getCurrentPose().getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stop();
  }

  public static Translation2d getFieldSpeeds(ChassisSpeeds chassisSpeeds, Rotation2d robotAngle) {
    return new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
        .rotateBy(robotAngle);
  }

  public static ChassisSpeeds getRobotSpeeds(Translation2d fieldSpeeds, ChassisSpeeds chassisSpeeds) {
    return new ChassisSpeeds(fieldSpeeds.getX(), fieldSpeeds.getY(), chassisSpeeds.omegaRadiansPerSecond);
  }
}