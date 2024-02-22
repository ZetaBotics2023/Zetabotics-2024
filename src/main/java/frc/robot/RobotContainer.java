// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Set up on the main driver station

// St up new branch
package frc.robot;

import frc.robot.Constants.AutonConfigurationConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.FieldOrientedDriveCommand;
import frc.robot.commands.LockSwerves;
import frc.robot.commands.ParallelRaceGroupCommand;
import frc.robot.commands.AutoCommands.GoToPositionCommands.PIDGoToPosition.AutoShootPositionCommand;
import frc.robot.commands.AutoCommands.GoToPositionCommands.PIDGoToPosition.GoToPoseAutonWhileShootingWithPIDs;
import frc.robot.commands.AutoCommands.GoToPositionCommands.PIDGoToPosition.GoToPositionWithPIDSAuto;
import frc.robot.commands.AutoCommands.GoToPositionCommands.PIDGoToPosition.GoToPositionAfterTimeWithPIDS;
import frc.robot.commands.ClimberCommands.ClimbDownDualCommand;
import frc.robot.commands.ClimberCommands.ClimbUpDualCommand;
import frc.robot.commands.IntakeCommands.HandOffToShooterAuton;
import frc.robot.commands.IntakeCommands.PickupFromGroundCommand;
import frc.robot.commands.IntakeCommands.ShootIntoAmpWithIntakeCommand;
import frc.robot.commands.ShooterCommands.RampShooterAtDifforentSpeedCommand;
import frc.robot.commands.ShooterCommands.ShootAtDiffSpeedCommand;
import frc.robot.commands.ShooterCommands.StopShooterCommand;
import frc.robot.subsystems.ClimberSubsystem.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSensorSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.PivotSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDSubsystem.RGBColor;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.utils.ButtonBoard;
import frc.robot.utils.GenerateAuto;
import frc.robot.utils.InTeleop;
import frc.robot.utils.MirrablePose2d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private final ClimberSubsystem m_climberSubsystem;
  private final LEDSubsystem m_ledSubsystem;

  private final PickupFromGroundCommand pickupFromGroundCommand;

  private final AutoShootCommand autoShootCommand;
  private final ShootIntoAmpWithIntakeCommand shootIntoAmpWithIntakeCommand;
  private final AutoShootPositionCommand autoShootPositionCommand;

  private final ShootAtDiffSpeedCommand shootAtDiffSpeedCommand;

  private final ClimbDownDualCommand climbDownDualCommand;
  private final ClimbUpDualCommand climbUpDualCommand;

  XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);
  ButtonBoard m_buttonBoard = new ButtonBoard(OperatorConstants.kButtonBoardPort);
  XboxController m_buttonBoardAlternative = new XboxController(OperatorConstants.kButtonBoardAltPort); // In the case that our button board is unusable, we will use a backup controller

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
        this.m_climberSubsystem = new ClimberSubsystem(false, true);
        this.m_ledSubsystem = new LEDSubsystem();

        this.m_ledSubsystem.setDefaultCommand(
          Commands.runOnce(() -> {this.m_ledSubsystem.setSolidColor(RGBColor.Orange.color);
          }, this.m_ledSubsystem));

    // Config Commands
    this.pickupFromGroundCommand = new PickupFromGroundCommand(
        this.m_intakeSubsystem, this.m_pivotSubsystem, this.m_intakeSensorSubsystem, this.m_ledSubsystem);

    this.shootIntoAmpWithIntakeCommand = new ShootIntoAmpWithIntakeCommand(
      this.m_intakeSubsystem, this.m_pivotSubsystem, this.m_intakeSensorSubsystem);

    this.autoShootCommand = new AutoShootCommand(this.m_shooterSubsystem, 
      this.m_intakeSubsystem, this.m_pivotSubsystem, this.m_intakeSensorSubsystem);

    this.autoShootPositionCommand = new AutoShootPositionCommand(m_driveSubsystem,
     m_shooterSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem);

     this.shootAtDiffSpeedCommand = new ShootAtDiffSpeedCommand(m_shooterSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem);

    this.climbDownDualCommand = new ClimbDownDualCommand(m_climberSubsystem);
    this.climbUpDualCommand = new ClimbUpDualCommand(m_climberSubsystem);


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
    this.autonSelector.addOption("Left:ShootPreloadedLeft", "Left:ShootPreloadedLeft");
    this.autonSelector.addOption("Left:ShootPreloadedLeftCenter", "Left:ShootPreloadedLeftCenter");
    this.autonSelector.addOption("Left:ShootPreloadedLeftCenterRight", "Left:ShootPreloadedLeftCenterRight");

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

    // *** Button monkey controls begin here! ***

    // pickupFromGroundCommand
    m_buttonBoard.bindToButton(0, ButtonBoard.Button.kBlue, this.pickupFromGroundCommand, Commands.runOnce(this.pickupFromGroundCommand::cancel));
    final JoystickButton pickUpFromGround = new JoystickButton(m_buttonBoardAlternative, XboxController.Button.kX.value);
    pickUpFromGround.onTrue(this.pickupFromGroundCommand);
    pickUpFromGround.onFalse(Commands.runOnce(this.pickupFromGroundCommand::cancel));

    // autoShootCommand
    m_buttonBoard.bindToButton(0, ButtonBoard.Button.kGreen, this.autoShootCommand, Commands.runOnce(this.autoShootCommand::cancel));
    final JoystickButton shootNote = new JoystickButton(m_buttonBoardAlternative, XboxController.Button.kB.value);
    shootNote.onTrue(this.autoShootCommand);
    shootNote.onFalse(Commands.runOnce(this.autoShootCommand::cancel));

    // run
    m_buttonBoard.bindToButton(0, ButtonBoard.Button.kRed, this.shootAtDiffSpeedCommand, Commands.runOnce(this.shootAtDiffSpeedCommand::cancel));
    final JoystickButton rampShooter = new JoystickButton(m_buttonBoardAlternative, XboxController.Button.kLeftBumper.value);
    rampShooter.onTrue(this.shootAtDiffSpeedCommand);
    rampShooter.onFalse(Commands.runOnce(this.shootAtDiffSpeedCommand::cancel));

    // autoShootPositionCommand
    m_buttonBoard.bindToAxis(0, m_buttonBoard.getController()::getPOV, 90, this.autoShootPositionCommand, Commands.runOnce(this.autoShootPositionCommand::cancel));
    final JoystickButton shootNoteAutoPose = new JoystickButton(m_buttonBoardAlternative, XboxController.Button.kA.value);
    shootNoteAutoPose.onTrue(this.autoShootPositionCommand);
    shootNoteAutoPose.onFalse(Commands.runOnce(this.autoShootPositionCommand::cancel));   

    // shootIntoAmpIntakeCommand
    m_buttonBoard.bindToButton(0, ButtonBoard.Button.kYellow, this.shootIntoAmpWithIntakeCommand, Commands.runOnce(this.shootIntoAmpWithIntakeCommand::cancel));
    final JoystickButton shootNoteIntoAmpWithIntake = new JoystickButton(m_buttonBoardAlternative, XboxController.Button.kY.value);
    shootNoteIntoAmpWithIntake.onTrue(this.shootIntoAmpWithIntakeCommand);
    shootNoteIntoAmpWithIntake.onFalse(Commands.runOnce(this.shootIntoAmpWithIntakeCommand::cancel));
  
    // climbUpDualCommand
    m_buttonBoard.bindToAxis(0, m_buttonBoard.getController()::getPOV, 180, this.climbUpDualCommand, Commands.runOnce(this.climbUpDualCommand::cancel));
    final JoystickButton moveClimbersUp = new JoystickButton(m_buttonBoardAlternative, XboxController.Button.kLeftBumper.value);
    moveClimbersUp.onTrue(this.climbUpDualCommand);
    moveClimbersUp.onFalse(Commands.runOnce(this.climbUpDualCommand::cancel));

    // climbDownDualCommand
    m_buttonBoard.bindToAxis(0, m_buttonBoard.getController()::getPOV, 0, this.climbDownDualCommand, Commands.runOnce(this.climbDownDualCommand::cancel));
    final JoystickButton moveClimbersDown = new JoystickButton(m_buttonBoardAlternative, XboxController.Button.kRightBumper.value);
    moveClimbersDown.onTrue(this.climbDownDualCommand);
    moveClimbersDown.onFalse(Commands.runOnce(this.climbDownDualCommand::cancel));
  }
  
  /*
   * Creates and returns a command that will execute the selected autonomous from SmartDashboard 
   */
  public Command getAutonomousCommand() {
    InTeleop.inTeleop = false;
    return configureAutons(this.autonSelector.getSelected());
  }

  /*
   * Changes the value of a joystick axis to:
   * - Apply deadband
   * - Square it
   */
  private static double modifyAxis(double value) {
    // Deadband
    value = MathUtil.applyDeadband(value, OperatorConstants.kDeadband);
    // Square the axis
    value = Math.copySign(value * value, value);
    return value;
  }

  /*
   * Popualtes the list of robot positions in AutonConfigurationConstants to have usable field coordinates
   */
  public void configureAutonPoints() {
    AutonConfigurationConstants.robotPositions.put("LeftNoteShootPose", new MirrablePose2d(new Pose2d(1.7, 7, Rotation2d.fromDegrees(35)), !AutonConfigurationConstants.kIsBlueAlliance));
    AutonConfigurationConstants.robotPositions.put("CenterNoteShootPose", new MirrablePose2d(new Pose2d(2.00, 5.55, new Rotation2d(0)), !AutonConfigurationConstants.kIsBlueAlliance));
    AutonConfigurationConstants.robotPositions.put("RightNoteShootPose", new MirrablePose2d(new Pose2d(2.00, 4.15,  Rotation2d.fromDegrees(-45)), !AutonConfigurationConstants.kIsBlueAlliance));

    AutonConfigurationConstants.robotPositions.put("LeftNoteIntakePose", new MirrablePose2d(new Pose2d(2.5, 7.00, new Rotation2d(0)), !AutonConfigurationConstants.kIsBlueAlliance));
    AutonConfigurationConstants.robotPositions.put("CenterNoteIntakePose", new MirrablePose2d(new Pose2d(2.5, 5.55, new Rotation2d(0)), !AutonConfigurationConstants.kIsBlueAlliance));
    AutonConfigurationConstants.robotPositions.put("RightNoteIntakePose", new MirrablePose2d(new Pose2d(2.5, 4.15, new Rotation2d(0)), !AutonConfigurationConstants.kIsBlueAlliance));
    

    AutonConfigurationConstants.robotPositions.put("LeftNoteIntakeZero", new MirrablePose2d(new Pose2d(1.7, 7.00, new Rotation2d(0)), !AutonConfigurationConstants.kIsBlueAlliance));
    AutonConfigurationConstants.robotPositions.put("RightNoteIntakeZero", new MirrablePose2d(new Pose2d(2, 4.15, new Rotation2d(0)), !AutonConfigurationConstants.kIsBlueAlliance));

    //AutonConfigurationConstatns.robotPositions.put("LeftNoteLeavePose", new MirrablePose2d(new Pose2d(2.20, 7.00, new Rotation2d(0)), !AutonConfigurationConstatns.kIsBlueAlience));
    //AutonConfigurationConstatns.robotPositions.put("CenterNoteLeavePose", new MirrablePose2d(new Pose2d(2.20, 5.55, new Rotation2d(0)), !AutonConfigurationConstatns.kIsBlueAlience));
    //AutonConfigurationConstatns.robotPositions.put("RightNoteLeavePose", new MirrablePose2d(new Pose2d(2.20, 4.15, new Rotation2d(0)), !AutonConfigurationConstatns.kIsBlueAlience));
  }

  /*
   * Given the name of an auton, populates the corresponding autonomous object in AutonConfigurationConstants
   */
  public Command configureAutons(String autonName) {
    switch (autonName.split(":")[0]) {
      case "Left":
          this.m_driveSubsystem.setRobotPose(new Pose2d(AutonConfigurationConstants.kLeftStartingPose.getX(),
          AutonConfigurationConstants.kLeftStartingPose.getY(), AutonConfigurationConstants.kLeftStartingPose.getRotation()));
        break;
      
      case "Center":
          this.m_driveSubsystem.setRobotPose(new Pose2d(AutonConfigurationConstants.kCenterStartingPose.getX(),
          AutonConfigurationConstants.kCenterStartingPose.getY(), AutonConfigurationConstants.kCenterStartingPose.getRotation()));
        break;
      
      case "Right": 
          this.m_driveSubsystem.setRobotPose(new Pose2d(AutonConfigurationConstants.kRightStartingPose.getX(),
          AutonConfigurationConstants.kRightStartingPose.getY(), AutonConfigurationConstants.kRightStartingPose.getRotation()));
        break;

      default:
        break;
    }
    switch(autonName) {
      case "Left:ShootPreloaded":
        AutonConfigurationConstants.kLeft_ShootPreloaded.add(new RampShooterAtDifforentSpeedCommand(m_shooterSubsystem));
        AutonConfigurationConstants.kLeft_ShootPreloaded.add(createGoToPositionCommand("LeftNoteShootPose"));
        AutonConfigurationConstants.kLeft_ShootPreloaded.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem,
            m_intakeSensorSubsystem));
        AutonConfigurationConstants.kLeft_ShootPreloaded.add(new StopShooterCommand(m_shooterSubsystem));

        return GenerateAuto.generateAuto(autonName, AutonConfigurationConstants.kLeft_ShootPreloaded);
      case "Left:ShootPreloadedLeft":
        AutonConfigurationConstants.kLeft_ShootPreloadedLeft.add(new RampShooterAtDifforentSpeedCommand(m_shooterSubsystem));
        AutonConfigurationConstants.kLeft_ShootPreloadedLeft.add(createGoToPositionCommand("LeftNoteShootPose"));
        AutonConfigurationConstants.kLeft_ShootPreloadedLeft.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
        AutonConfigurationConstants.kLeft_ShootPreloadedLeft.add(createGoToPositionCommand("LeftNoteIntakeZero"));
        AutonConfigurationConstants.kLeft_ShootPreloadedLeft.add(createIntakeCommand("LeftNoteIntakePose", AutonConfigurationConstants.kLeftNoteIntakeDownTime));
        AutonConfigurationConstants.kLeft_ShootPreloadedLeft.add(createGoToPositionCommand("LeftNoteShootPose"));
        AutonConfigurationConstants.kLeft_ShootPreloadedLeft.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
        AutonConfigurationConstants.kLeft_ShootPreloadedLeft.add(new StopShooterCommand(m_shooterSubsystem));
    
        return GenerateAuto.generateAuto(autonName, AutonConfigurationConstants.kLeft_ShootPreloadedLeft);
      
      case "Left:ShootPreloadedLeftCenter":
        AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter.add(new RampShooterAtDifforentSpeedCommand(m_shooterSubsystem));
        AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter.add(createGoToPositionCommand("LeftNoteShootPose"));
        AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
        AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter.add(createGoToPositionCommand("LeftNoteIntakeZero"));
        AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter.add(createIntakeCommand("LeftNoteIntakePose", AutonConfigurationConstants.kLeftNoteIntakeDownTime));
        AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter.add(createGoToPositionCommand("CenterNoteShootPose"));
        AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
        AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter.add(createIntakeCommand("CenterNoteIntakePose", AutonConfigurationConstants.kCenterNoteIntakeDownTime));
        AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter.add(createGoToPositionCommand("CenterNoteShootPose"));
        AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
        AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter.add(new StopShooterCommand(m_shooterSubsystem));

        return GenerateAuto.generateAuto(autonName, AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter);
      
      case "Left:ShootPreloadedLeftCenterRight":
        AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterRight.add(new RampShooterAtDifforentSpeedCommand(m_shooterSubsystem));
        AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterRight.add(createGoToPositionCommand("LeftNoteShootPose"));
        AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterRight.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
        AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterRight.add(createGoToPositionCommand("LeftNoteIntakeZero"));
        AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterRight.add(createIntakeCommand("LeftNoteIntakePose", AutonConfigurationConstants.kLeftNoteIntakeDownTime));
        AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterRight.add(createGoToPositionCommand("CenterNoteShootPose"));
        AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterRight.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
        AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterRight.add(createIntakeCommand("CenterNoteIntakePose", AutonConfigurationConstants.kCenterNoteIntakeDownTime));
        AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterRight.add(createGoToPositionCommand("RightNoteShootPose"));
        AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterRight.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
        AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterRight.add(createGoToPositionCommand("RightNoteIntakeZero"));
        AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterRight.add(createIntakeCommand("RightNoteIntakePose", AutonConfigurationConstants.kRightNoteIntakeDownTime));
        AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterRight.add(createGoToPositionCommand("RightNoteShootPose"));
        AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterRight.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
        AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterRight.add(new StopShooterCommand(m_shooterSubsystem));

        return GenerateAuto.generateAuto(autonName, AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterRight);
      }

      return null;
  }

  /*
   * Given the name of a target field position and a time to wait before doing so, create a command that runs the intake and moves to the position
   */
  public Command createIntakeCommand(String poseName, double waitTime) {
    return new ParallelRaceGroupCommand(new PickupFromGroundCommand(m_intakeSubsystem, m_pivotSubsystem,
     m_intakeSensorSubsystem, this.m_ledSubsystem),
      new GoToPositionAfterTimeWithPIDS(
        new GoToPositionWithPIDSAuto(m_driveSubsystem, 
        AutonConfigurationConstants.robotPositions.get(poseName)), waitTime));
  }

  /*
   * Given the name of a target field position, create a command that moves to the position
   */
  public GoToPositionWithPIDSAuto createGoToPositionCommand(String poseName) {
    return new GoToPositionWithPIDSAuto(this.m_driveSubsystem,
          AutonConfigurationConstants.robotPositions.get(poseName));
  }

  /*
   * Given the name of a target field position and a percentage at which to start shooting, create a command that moves to a position and shoots during transit
   */
   public GoToPoseAutonWhileShootingWithPIDs createGoToPoseAutonWhileShooting(String poseName, double percentToPose) {
    return new GoToPoseAutonWhileShootingWithPIDs(
          this.m_driveSubsystem,
          new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem),
          AutonConfigurationConstants.robotPositions.get(poseName),
          percentToPose
    );    
  }
}


