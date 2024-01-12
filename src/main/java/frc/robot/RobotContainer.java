// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Set up on the main driver station

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.FieldOrientedDriveCommand;
import frc.robot.commands.FollowAutonomousPath;
import frc.robot.commands.LockSwerves;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.subsystems.SwerveDrive.PoseEstimatorSubsystem;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final DriveSubsystem m_driveSubsystem;
  private final LockSwerves lockSwerves;
  private final FieldOrientedDriveCommand fieldOrientedDriveCommand;
  private SendableChooser<Command> autoChooser;

  XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    this.m_driveSubsystem = new DriveSubsystem();
    this.lockSwerves = new LockSwerves(m_driveSubsystem);

    this.fieldOrientedDriveCommand = new FieldOrientedDriveCommand(
      m_driveSubsystem,
      () -> -modifyAxis(m_driverController.getLeftY()),
      () -> -modifyAxis(m_driverController.getLeftX()),
      () -> -modifyAxis(m_driverController.getRightX())
      );

    m_driveSubsystem.setDefaultCommand(fieldOrientedDriveCommand);


   
    configureBindings();
/* 
    Supplier<Pose2d> currentPoseSupplier = () -> m_driveSubsystem.getCurrentPose();
    Consumer<Pose2d> resetPoseConsumer = (Pose2d) -> poseEstimator.setCurrentPose(Pose2d);
    Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier = () -> m_driveSubsystem.getChassisSpeeds();
    Consumer<ChassisSpeeds> robotRelativeSpeedsConsumer = (newChassisSpeeds) -> m_driveSubsystem.setModuleStates(SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(newChassisSpeeds));
    AutoBuilder.configureHolonomic(currentPoseSupplier, resetPoseConsumer, robotRelativeSpeedsSupplier, robotRelativeSpeedsConsumer, null, m_driveSubsystem);


    NamedCommands.registerCommands(Constants.AutoConstants.namedEventMap);

    this.autoChooser = AutoBuilder.buildAutoChooser();

    this.autoChooser.addOption("Example Auton", FollowAutonomousPath.followPathCommand(poseEstimator, m_driveSubsystem, "benjaminsmom"));

    // Another option that allows you to specify the default auto by its name
    autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
    */
  }

 
  private void configureBindings() {
    final JoystickButton lockSwerves =  new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
    lockSwerves.onTrue(Commands.runOnce(this.lockSwerves::schedule));
    lockSwerves.onFalse(Commands.runOnce(this.lockSwerves::cancel));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  

  private static double modifyAxis(double value) {
    // Deadband
    value = MathUtil.applyDeadband(value, OperatorConstants.kDeadband);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  public void onAllianceChanged(Alliance currentAlliance) {
    this.m_driveSubsystem.getPoseEstimatorSubsystem().setAlliance(currentAlliance);
  }
}
