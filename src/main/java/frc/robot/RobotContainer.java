// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Set up on the main driver station

// St up new branch
package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutonConfigurationConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.FieldOrientedDriveCommand;
import frc.robot.commands.LockSwerves;
import frc.robot.commands.AutoCommands.AutoShootPositionCommand;
import frc.robot.commands.AutoCommands.FollowAutonomousPath;
import frc.robot.commands.AutoCommands.GoToPosition;
import frc.robot.commands.AutoCommands.GoToPositionAuton;
import frc.robot.commands.AutoCommands.TestCommand;
import frc.robot.commands.IntakeCommands.GoToLocation;
import frc.robot.commands.IntakeCommands.HandOffToShooterAuton;
import frc.robot.commands.IntakeCommands.HandOffToShooterCommand;
import frc.robot.commands.IntakeCommands.PickupFromGroundCommand;
import frc.robot.commands.IntakeCommands.ShootIntoAmpWithIntakeCommand;
import frc.robot.commands.ShooterCommands.RampShooterAtDifforentSpeedAutonCommand;
import frc.robot.commands.ShooterCommands.RampShooterAtDifforentSpeedCommand;
import frc.robot.commands.ShooterCommands.StopShooterCommand;
import frc.robot.subsystems.IntakeSubsystem.IntakeSensorSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.utils.GenerateAuto;
import frc.robot.utils.MirrablePose2d;

import java.security.GeneralSecurityException;
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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  private SendableChooser<String> autonSelector;

  private final IntakeSubsystem m_intakeSubsystem;
  private final IntakeSensorSubsystem m_intakeSensorSubsystem;
  private final PivotSubsystem m_pivotSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;

  private final PickupFromGroundCommand pickupFromGroundCommand;
  private final HandOffToShooterCommand handOffToShooterCommand;

  private final RampShooterAtDifforentSpeedCommand rampShooterAtDifforentSpeedCommand;
  private final StopShooterCommand stopShooterCommand;

  private final AutoShootCommand autoShootCommand;
  private final ShootIntoAmpWithIntakeCommand shootIntoAmpWithIntakeCommand;
  private final AutoShootPositionCommand autoShootPositionCommand;

  XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    /*
     * Adding our commands before we instantiat our drive subsystem,
     * so they are added before the autobuilder is configured.
     */
    this.autonSelector = new SendableChooser<>();
    this.m_driveSubsystem = new DriveSubsystem();

    this.fieldOrientedDriveCommand = new FieldOrientedDriveCommand(
        m_driveSubsystem,
        () -> -modifyAxis(m_driverController.getLeftY()),
        () -> -modifyAxis(m_driverController.getLeftX()),
        () -> -modifyAxis(m_driverController.getRightX()));

    this.m_driveSubsystem.setDefaultCommand(fieldOrientedDriveCommand);
    this.m_intakeSubsystem = new IntakeSubsystem(false);
    this.m_pivotSubsystem = new PivotSubsystem(true);
    this.m_intakeSensorSubsystem = new IntakeSensorSubsystem();
    this.m_shooterSubsystem = new ShooterSubsystem(false, true);


    // Config Commands
    this.pickupFromGroundCommand = new PickupFromGroundCommand(
        this.m_intakeSubsystem, this.m_pivotSubsystem, this.m_intakeSensorSubsystem);

    this.handOffToShooterCommand = new HandOffToShooterCommand(
        this.m_intakeSubsystem, this.m_pivotSubsystem, this.m_intakeSensorSubsystem);

    this.shootIntoAmpWithIntakeCommand = new ShootIntoAmpWithIntakeCommand(
      this.m_intakeSubsystem, this.m_pivotSubsystem, this.m_intakeSensorSubsystem);

    this.autoShootCommand = new AutoShootCommand(this.m_shooterSubsystem, 
      this.m_intakeSubsystem, this.m_pivotSubsystem, this.m_intakeSensorSubsystem);

    this.autoShootPositionCommand = new AutoShootPositionCommand(m_driveSubsystem,
     m_shooterSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem);
     
    this.rampShooterAtDifforentSpeedCommand = new RampShooterAtDifforentSpeedCommand(
      this.m_shooterSubsystem);

    this.stopShooterCommand = new StopShooterCommand(this.m_shooterSubsystem);

    this.lockSwerves = new LockSwerves(m_driveSubsystem);

    // End Command Config
    
    //AutoConstants.namedEventMap.put("PickUpFromGround", this.pickupFromGroundCommand);
    //NamedCommands.registerCommands(AutoConstants.namedEventMap);

    //this.autonSelector = AutoBuilder.buildAutoChooser();s
    // Autos go here
    //SmartDashboard.putData("Auton Selector", autonSelector);
    configureBindings();
    configureAutonPoints();
    this.autonSelector.addOption("Left:ShootPreloaded", "Left:ShootPreloaded");
    this.autonSelector.addOption("Left:ShootPreloadeds", "Left:ShootPreloadedLeft");

    SmartDashboard.putData("Auto Selector", this.autonSelector);
  }

  private void configureBindings() {
    final JoystickButton lockSwerves = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
    lockSwerves.onTrue(Commands.runOnce(this.lockSwerves::schedule));
    lockSwerves.onFalse(Commands.runOnce(this.lockSwerves::cancel));

    final JoystickButton resetHeading = new JoystickButton(m_driverController, XboxController.Button.kY.value);
    resetHeading.onTrue(Commands.runOnce(this.m_driveSubsystem::resetRobotHeading));

    final JoystickButton resetOdometry = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);
    resetOdometry.onTrue(Commands.runOnce(this.m_driveSubsystem::resetRobotPose));

    final JoystickButton pickUpFromGround = new JoystickButton(m_driverController, XboxController.Button.kX.value);
    pickUpFromGround.onTrue(this.pickupFromGroundCommand);
    pickUpFromGround.onFalse(Commands.runOnce(this.pickupFromGroundCommand::cancel));

    final JoystickButton shootNote = new JoystickButton(m_driverController, XboxController.Button.kB.value);
    shootNote.onTrue(this.autoShootCommand);
    shootNote.onFalse(Commands.runOnce(this.autoShootCommand::cancel));

    final JoystickButton shootNoteAutoPose = new JoystickButton(m_driverController, XboxController.Button.kA.value);
    shootNoteAutoPose.onTrue(this.autoShootPositionCommand);
    shootNoteAutoPose.onFalse(Commands.runOnce(this.autoShootPositionCommand::cancel));   
  }
  
  public Command getAutonomousCommand() {
    return configureAutons(autonSelector.getSelected());
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = MathUtil.applyDeadband(value, OperatorConstants.kDeadband);
    // Square the axis
    value = Math.copySign(value * value, value);
    return value;
  }

  public void configureAutonPoints() {
    AutonConfigurationConstants.robotPositions.put("LeftNoteShootPose", new MirrablePose2d(new Pose2d(1.8, 7, Rotation2d.fromDegrees(35)), !AutonConfigurationConstants.kIsBlueAlience));
    AutonConfigurationConstants.robotPositions.put("CenterNoteShootPose", new MirrablePose2d(new Pose2d(2.00, 5.55, new Rotation2d(0)), !AutonConfigurationConstants.kIsBlueAlience));
    AutonConfigurationConstants.robotPositions.put("RightNoteShootPose", new MirrablePose2d(new Pose2d(2.00, 4.15,  Rotation2d.fromDegrees(-45)), !AutonConfigurationConstants.kIsBlueAlience));

    AutonConfigurationConstants.robotPositions.put("LeftNoteIntakePose", new MirrablePose2d(new Pose2d(2.125, 7.00, new Rotation2d(0)), !AutonConfigurationConstants.kIsBlueAlience));
    AutonConfigurationConstants.robotPositions.put("CenterNoteIntakePose", new MirrablePose2d(new Pose2d(2.3, 5.55, new Rotation2d(0)), !AutonConfigurationConstants.kIsBlueAlience));
    AutonConfigurationConstants.robotPositions.put("RightNoteIntakePose", new MirrablePose2d(new Pose2d(2.3, 4.15, new Rotation2d(0)), !AutonConfigurationConstants.kIsBlueAlience));
    
    //AutonConfigurationConstatns.robotPositions.put("LeftNoteLeavePose", new MirrablePose2d(new Pose2d(2.20, 7.00, new Rotation2d(0)), !AutonConfigurationConstatns.kIsBlueAlience));
    //AutonConfigurationConstatns.robotPositions.put("CenterNoteLeavePose", new MirrablePose2d(new Pose2d(2.20, 5.55, new Rotation2d(0)), !AutonConfigurationConstatns.kIsBlueAlience));
    //AutonConfigurationConstatns.robotPositions.put("RightNoteLeavePose", new MirrablePose2d(new Pose2d(2.20, 4.15, new Rotation2d(0)), !AutonConfigurationConstatns.kIsBlueAlience));
  }

  public Command configureAutons(String autonName) {
    switch(autonName) {
      case "Left:ShootPreloaded":
      AutonConfigurationConstants.kLeft_ShootPreloaded.add(new RampShooterAtDifforentSpeedAutonCommand(m_shooterSubsystem));
      AutonConfigurationConstants.kLeft_ShootPreloaded.add(new GoToPositionAuton(m_driveSubsystem,
          AutonConfigurationConstants.robotPositions.get("LeftNoteShootPose")));
      AutonConfigurationConstants.kLeft_ShootPreloaded.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
      AutonConfigurationConstants.kLeft_ShootPreloaded.add(new StopShooterCommand(m_shooterSubsystem));

      return GenerateAuto.generateAuto(AutonConfigurationConstants.kLeft_ShootPreloaded);
      case "Left:ShootPreloadedLeft":
      AutonConfigurationConstants.kLeft_ShootPreloadedLeft.add(new RampShooterAtDifforentSpeedAutonCommand(m_shooterSubsystem));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeft.add(new GoToPositionAuton(m_driveSubsystem,
      AutonConfigurationConstants.robotPositions.get("LeftNoteShootPose")));
          AutonConfigurationConstants.kLeft_ShootPreloadedLeft.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeft.add(new ParallelCommandGroup(
          new GoToPositionAuton(m_driveSubsystem, 
          AutonConfigurationConstants.robotPositions.get("LeftNoteIntakePose")),
          new PickupFromGroundCommand(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem)));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeft.add(new GoToPositionAuton(m_driveSubsystem,
          AutonConfigurationConstants.robotPositions.get("CenterNoteShootPose")));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeft.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeft.add(new StopShooterCommand(m_shooterSubsystem));
      
      return GenerateAuto.generateAuto(AutonConfigurationConstants.kLeft_ShootPreloadedLeft);
      }

      return null;
  }

}
