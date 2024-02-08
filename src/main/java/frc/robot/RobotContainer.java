// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Set up on the main driver station

// St up new branch
package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.FieldOrientedDriveCommand;
import frc.robot.commands.LockSwerves;
import frc.robot.commands.AutoCommands.FollowAutonomousPath;
import frc.robot.commands.AutoCommands.TestCommand;
import frc.robot.commands.IntakeCommands.GoToLocation;
import frc.robot.commands.IntakeCommands.HandOffToShooterCommand;
import frc.robot.commands.IntakeCommands.PickupFromGroundCommand;
import frc.robot.commands.IntakeCommands.ShootIntoAmpWithIntakeCommand;
import frc.robot.subsystems.IntakeSubsystem.IntakeSensorSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.PivotSubsystem;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final DriveSubsystem m_driveSubsystem;
  private final LockSwerves lockSwerves;
  private final FieldOrientedDriveCommand fieldOrientedDriveCommand;
  private SendableChooser<Command> autonSelector;

  private final IntakeSubsystem m_intakeSubsystem;
  private final IntakeSensorSubsystem m_intakeSensorSubsystem;
  private final PivotSubsystem m_pivotSubsystem;

  private final PickupFromGroundCommand pickupFromGroundCommand;
  private final HandOffToShooterCommand handOffToShooterCommand;
  private final ShootIntoAmpWithIntakeCommand shootIntoAmpWithIntakeCommand;

  private final GoToLocation goToLocation;

  XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    /*
     * Adding our commands before we instantiat our drive subsystem,
     * so they are added before the autobuilder is configured.
     */
   
    this.m_driveSubsystem = new DriveSubsystem();
    SmartDashboard.putBoolean("Ran Command", false);

    this.fieldOrientedDriveCommand = new FieldOrientedDriveCommand(
        m_driveSubsystem,
        () -> -modifyAxis(m_driverController.getLeftY()),
        () -> -modifyAxis(m_driverController.getLeftX()),
        () -> -modifyAxis(m_driverController.getRightX()));

    this.m_driveSubsystem.setDefaultCommand(fieldOrientedDriveCommand);
    this.m_intakeSubsystem = new IntakeSubsystem(false);
    this.m_pivotSubsystem = new PivotSubsystem(true);
    this.m_intakeSensorSubsystem = new IntakeSensorSubsystem();

    this.lockSwerves = new LockSwerves(m_driveSubsystem);

    // Config Commands
    this.pickupFromGroundCommand = new PickupFromGroundCommand(
        this.m_intakeSubsystem, this.m_pivotSubsystem, this.m_intakeSensorSubsystem);
    this.handOffToShooterCommand = new HandOffToShooterCommand(
        this.m_intakeSubsystem, this.m_pivotSubsystem, this.m_intakeSensorSubsystem);
    this.shootIntoAmpWithIntakeCommand = new ShootIntoAmpWithIntakeCommand(
      this.m_intakeSubsystem, this.m_pivotSubsystem, this.m_intakeSensorSubsystem);

    this.goToLocation = new GoToLocation(m_pivotSubsystem);
    
    configureBindings();
    //AutoConstants.namedEventMap.put("PrintCommand", new TestCommand());
//NamedCommands.registerCommands(AutoConstants.namedEventMap);

   // this.autonSelector = AutoBuilder.buildAutoChooser();
    // Autos go here
   // SmartDashboard.putData("Auton Selector", autonSelector);
  }

  private void configureBindings() {
    final JoystickButton lockSwerves = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
    lockSwerves.onTrue(Commands.runOnce(this.lockSwerves::schedule));
    lockSwerves.onFalse(Commands.runOnce(this.lockSwerves::cancel));
    final JoystickButton resetHeading = new JoystickButton(m_driverController, XboxController.Button.kY.value);
    resetHeading.onTrue(Commands.runOnce(this.m_driveSubsystem::resetRobotHeading));
    final JoystickButton resetOdometry = new JoystickButton(m_driverController, XboxController.Button.kA.value);
    resetOdometry.onTrue(Commands.runOnce(this.m_driveSubsystem::resetRobotPose));
    final JoystickButton pickUpFromGround = new JoystickButton(m_driverController, XboxController.Button.kX.value);
    pickUpFromGround.onTrue(this.pickupFromGroundCommand);
    final JoystickButton handOffToShooter = new JoystickButton(m_driverController, XboxController.Button.kB.value);
    //handOffToShooter.onTrue(this.handOffToShooterCommand);
    final JoystickButton shootIntoAmpWithIntake = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);
    //shootIntoAmpWithIntake.onTrue(this.shootIntoAmpWithIntakeCommand);
    handOffToShooter.onTrue(Commands.runOnce(this.goToLocation::schedule));
    handOffToShooter.onFalse(Commands.runOnce(this.goToLocation::cancel));

    
  }
  

  public Command getAutonomousCommand() {
    return autonSelector.getSelected();
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = MathUtil.applyDeadband(value, OperatorConstants.kDeadband);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  public void onAllianceChanged(Alliance currentAlliance) {

  }
}
