// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Set up on the main driver station

// St up new branch
package frc.robot;

import frc.robot.Constants.AutonConfigurationConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.FieldOrientedDriveCommand;
import frc.robot.commands.LockSwerves;
import frc.robot.commands.ParallelRaceGroupCommand;
import frc.robot.commands.RunCommandUtillConditionCommand;
import frc.robot.commands.SequentialGroupCommand;
import frc.robot.commands.AutoCommands.GoToPose;
import frc.robot.commands.AutoCommands.WaitCommandWrapper;
import frc.robot.commands.AutoCommands.AutoShootCommands.AutoShootPositionAmpCommand;
import frc.robot.commands.AutoCommands.AutoShootCommands.AutoShootPositionCenterCommand;
import frc.robot.commands.AutoCommands.AutoShootCommands.AutoShootPositionLeftCommand;
import frc.robot.commands.AutoCommands.AutoShootCommands.AutoShootPositionRightCommand;
import frc.robot.commands.AutoCommands.GoToPositionCommands.PIDGoToPosition.GoToPoseAutonWhileShootingWithPIDS;
import frc.robot.commands.AutoCommands.GoToPositionCommands.PIDGoToPosition.GoToPositionWithPIDSAuto;
import frc.robot.commands.AutoCommands.GoToPositionCommands.PIDGoToPosition.GoToPositionWithPIDSAutoCenter;
import frc.robot.commands.AutoCommands.GoToPositionCommands.PIDGoToPosition.GoToPositionAfterTimeWithPIDS;
import frc.robot.commands.AutoCommands.GoToPositionCommands.PIDGoToPosition.GoToPositionAfterTimeWithPIDSCenter;
import frc.robot.commands.ClimberCommands.ClimbDownLeftCommand;
import frc.robot.commands.ClimberCommands.ClimbDownRightCommand;
import frc.robot.commands.ClimberCommands.ClimbUpLeftCommand;
import frc.robot.commands.ClimberCommands.ClimbUpRightCommand;
import frc.robot.commands.IntakeCommands.HandOffToShooterAuton;
import frc.robot.commands.IntakeCommands.IntakeSpin;
import frc.robot.commands.IntakeCommands.PickupFromGroundCommand;
import frc.robot.commands.IntakeCommands.ShootIntoAmpWithIntakeCommand;
import frc.robot.commands.ShooterCommands.RampShooter;
import frc.robot.commands.ShooterCommands.RampShooterAtDifferentSpeedAutonCommand;
import frc.robot.commands.ShooterCommands.RampShooterAtDifforentSpeedCommand;
import frc.robot.commands.ShooterCommands.StopShooterCommand;
import frc.robot.subsystems.ClimberSubsystem.LeftClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.RightClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem.PivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSensorSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem.CTRELEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.utils.ButtonBoard;
import frc.robot.utils.GenerateAuto;
import frc.robot.utils.InTeleop;
import frc.robot.utils.MirrablePose2d;

import java.util.Optional;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
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
  private final LeftClimberSubsystem m_leftClimberSubsystem;
  private final RightClimberSubsystem m_rightClimberSubsystem;

  private final CTRELEDSubsystem m_ledSubsystem;

  private final PickupFromGroundCommand pickupFromGroundCommand;

  private final AutoShootCommand autoShootCommand;
  private final ShootIntoAmpWithIntakeCommand shootIntoAmpWithIntakeCommand;

  private final ClimbUpLeftCommand climbUpLeftCommand;
  private final ClimbDownLeftCommand climbDownLeftCommand;

  private final ClimbUpRightCommand climbUpRightCommand;
  private final ClimbDownRightCommand climbDownRightCommand;
  private final IntakeSpin intakeSpin;

  private XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);
  private ButtonBoard m_buttonBoard = new ButtonBoard(OperatorConstants.kButtonBoardPort);
 //private XboxController m_buttonBoardAlternative = new XboxController(OperatorConstants.kButtonBoardAltPort); // In the case that our button board is unusable, we will use a backup controller
  private AutoShootPositionLeftCommand autoShootPositionLeftCommand;
  private AutoShootPositionCenterCommand autoShootPositionCenterCommand;
  private AutoShootPositionRightCommand autoShootPositionRightCommand;
  private AutoShootPositionAmpCommand autoShootPositionAmpCommand;

  private RampShooter rampShooter;

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
        this.m_pivotSubsystem = new PivotSubsystem();
        this.m_intakeSensorSubsystem = new IntakeSensorSubsystem();
        this.m_shooterSubsystem = new ShooterSubsystem(false, true);
        this.m_leftClimberSubsystem = new LeftClimberSubsystem(true);
        this.m_rightClimberSubsystem = new RightClimberSubsystem(false);
        this.m_ledSubsystem = new CTRELEDSubsystem();
        this.m_ledSubsystem.setSolidColor(new int[] {0, 255, 255});
    // Config Commands
    
    this.pickupFromGroundCommand = new PickupFromGroundCommand(
        this.m_intakeSubsystem, this.m_pivotSubsystem, this.m_intakeSensorSubsystem, this.m_ledSubsystem);

    this.shootIntoAmpWithIntakeCommand = new ShootIntoAmpWithIntakeCommand(
      this.m_intakeSubsystem, this.m_pivotSubsystem, this.m_intakeSensorSubsystem);

    this.autoShootCommand = new AutoShootCommand(this.m_shooterSubsystem, 
      this.m_intakeSubsystem, this.m_pivotSubsystem, this.m_intakeSensorSubsystem);

      this.autoShootPositionLeftCommand = new AutoShootPositionLeftCommand(m_driveSubsystem,
     m_shooterSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem, this.m_ledSubsystem);

    this.autoShootPositionCenterCommand = new AutoShootPositionCenterCommand(m_driveSubsystem,
     m_shooterSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem, this.m_ledSubsystem);

     this.autoShootPositionRightCommand = new AutoShootPositionRightCommand(m_driveSubsystem,
     m_shooterSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem, this.m_ledSubsystem);

    this.autoShootPositionAmpCommand = new AutoShootPositionAmpCommand(m_driveSubsystem,
     m_shooterSubsystem, m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem, this.m_ledSubsystem);

    this.climbUpLeftCommand = new ClimbUpLeftCommand(this.m_leftClimberSubsystem);
    this.climbDownLeftCommand = new ClimbDownLeftCommand(this.m_leftClimberSubsystem);

    this.climbUpRightCommand = new ClimbUpRightCommand(this.m_rightClimberSubsystem);
    this.climbDownRightCommand = new ClimbDownRightCommand(this.m_rightClimberSubsystem);

    this.lockSwerves = new LockSwerves(m_driveSubsystem);

    this.rampShooter = new RampShooter(m_shooterSubsystem);
    this.intakeSpin = new IntakeSpin(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem);

    // End Command Config
    
    //AutoConstants.namedEventMap.put("PickUpFromGround", this.pickupFromGroundCommand);
    //NamedCommands.registerCommands(AutoConstants.namedEventMap);

    //this.autonSelector = AutoBuilder.buildAutoChooser();s
    // Autos go here
    ////SmartDashBoard.putData("Auton Selector", autonSelector);
    configureBindings();
    configureAutonPoints();
    this.autonSelector.addOption("Left:ShootPreloaded", "Left:ShootPreloaded");
    this.autonSelector.addOption("Left:ShootPreloadedLeft", "Left:ShootPreloadedLeft");
    this.autonSelector.addOption("Left:ShootPreloadedLeftCenter", "Left:ShootPreloadedLeftCenter");
    //this.autonSelector.addOption("Left:ShootPreloadedLeftCenterRight", "Left:ShootPreloadedLeftCenterRight");

    this.autonSelector.addOption("Right:ShootPreloaded", "Right:ShootPreloaded");
    this.autonSelector.addOption("Right:ShootPreloadedRight", "Right:ShootPreloadedRight");
    this.autonSelector.addOption("Right:ShootPreloadedRightCenter", "Right:ShootPreloadedRightCenter");

    this.autonSelector.addOption("Left:ShootPreloadedFarFarLeftFarLeft", "Left:ShootPreloadedFarFarLeftFarLeft");

    this.autonSelector.addOption("Center:ShootPreloadedCenter", "Center:ShootPreloadedCenter");
    this.autonSelector.addOption("Center:ShootPreloadedCenterFarFarLeft", "Center:ShootPreloadedCenterFarFarLeft");

    this.autonSelector.setDefaultOption("Left:ShootPreloadedLeftCenter", "Left:ShootPreloadedLeftCenter");
    SmartDashboard.putData("Auto Selector", this.autonSelector);
  }

  private void configureBindings() {
    final JoystickButton lockSwerves = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
    lockSwerves.onTrue(Commands.runOnce(this.lockSwerves::schedule));
    lockSwerves.onFalse(Commands.runOnce(this.lockSwerves::cancel));

    //final JoystickButton resetHeading = new JoystickButton(m_driverController, XboxController.Button.kY.value);
    //resetHeading.onTrue(Commands.runOnce(this.m_driveSubsystem::resetRobotHeading));

    final JoystickButton resetOdometry = new JoystickButton(m_driverController, XboxController.Button.kY.value);
    resetOdometry.onTrue(Commands.runOnce(this.m_driveSubsystem::resetRobotPose));

    // *** Button monkey controls begin here! ***

    // autoShootCommand
    
    // Ramp Shooter: Bottem Left Black
    // Normal Shoot: Up D-Pad
    // Left: D-Pad
    // Center: D-Pad
    // Right: D-Pad
    // Intake Down: Top Left Black

    // Shoot Into Amp: Top Right White
    // Both Climb Up: Red
    // Both Climb Down: Green
    // Left Climb Up: Top Right Black
    // Left Climb Down: Bottom Right Black
    // Right Climb Up: Yellow 
    // Left Climb Down: Blue

    m_buttonBoard.bindToLeftTriggure(0, this.rampShooter, null);
    //final JoystickButton rampShooter = new JoystickButton(m_buttonBoardAlternative, XboxController.Button.kY.value);
    //rampShooter.onTrue(this.shootAtDiffSpeedCommand);
    //rampShooter.onFalse(Commands.runOnce(this.shootAtDiffSpeedCommand::cancel)); 

    this.m_buttonBoard.bindToPOV(0, 0, autoShootCommand, Commands.runOnce(autoShootCommand::cancel));
    //final JoystickButton shootNote = new JoystickButton(m_buttonBoardAlternative, XboxController.Button.kB.value);
    //shootNote.onTrue(this.autoShootCommand);
    //shootNote.onFalse(Commands.runOnce(this.autoShootCommand::cancel));

    this.m_buttonBoard.bindToPOV(0, 270, autoShootPositionLeftCommand, Commands.runOnce(autoShootPositionLeftCommand::cancel));

    this.m_buttonBoard.bindToPOV(0, 180, autoShootPositionCenterCommand, Commands.runOnce(autoShootPositionCenterCommand::cancel));
    //final JoystickButton shootNoteAutoPoseCenter = new JoystickButton(m_buttonBoardAlternative, XboxController.Button.kA.value);
    //shootNoteAutoPoseCenter.onTrue(this.autoShootPositionLeftCommand);
    //shootNoteAutoPoseCenter.onFalse(Commands.runOnce(this.autoShootPositionLeftCommand::cancel));

    this.m_buttonBoard.bindToPOV(0, 90, autoShootPositionRightCommand, Commands.runOnce(autoShootPositionRightCommand::cancel));
    this.m_buttonBoard.bindToButton(0, ButtonBoard.Button.kTopRightBlack, autoShootPositionAmpCommand, Commands.runOnce(autoShootPositionAmpCommand::cancel));

    // pickupFromGroundCommand
    m_buttonBoard.bindToRightTriggure(0, this.pickupFromGroundCommand, Commands.runOnce(this.pickupFromGroundCommand::cancel));
    //final JoystickButton pickUpFromGround = new JoystickButton(m_buttonBoardAlternative, XboxController.Button.kLeftBumper.value);
    //pickUpFromGround.onTrue(this.pickupFromGroundCommand);
    //pickUpFromGround.onFalse(Commands.runOnce(this.pickupFromGroundCommand::cancel));

    m_buttonBoard.bindToButton(0, ButtonBoard.Button.kBlackStart, this.shootIntoAmpWithIntakeCommand, Commands.runOnce(this.shootIntoAmpWithIntakeCommand::cancel));

    m_buttonBoard.bindToButton(0,  ButtonBoard.Button.kYellow, this.climbUpLeftCommand, Commands.runOnce(this.climbUpLeftCommand::cancel));
    m_buttonBoard.bindToButton(0,  ButtonBoard.Button.kBlue, this.climbDownLeftCommand, Commands.runOnce(this.climbDownLeftCommand::cancel));

    m_buttonBoard.bindToButton(0,  ButtonBoard.Button.kRed, this.climbUpRightCommand, Commands.runOnce(this.climbUpRightCommand::cancel));
    m_buttonBoard.bindToButton(0,  ButtonBoard.Button.kGreen, this.climbDownRightCommand, Commands.runOnce(this.climbDownRightCommand::cancel));
    

    m_buttonBoard.bindToButton(0,  ButtonBoard.Button.kWhiteLeft, this.intakeSpin, Commands.runOnce(this.intakeSpin::cancel));
    //m_buttonBoard.bindToButton(0,  ButtonBoard.Button.kWhiteRight, Commands.runOnce(() -> {ShooterConstants.kShooterRPMChange += 50;}), null);
    //m_buttonBoard.bindToButton(0,  ButtonBoard.Button.kBlackSelect, Commands.runOnce(() -> {ShooterConstants.kShooterRPMChange = 0;}), null);

  }
  
  /*
   * Creates and returns a command that will execute the selected autonomous from //SmartDashBoard 
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
    //  AutonConfigurationConstants.kIsBlueAlience ? 1.85 : 2
    //  AutonConfigurationConstants.kIsBlueAlience ? 2:  1.85
    //  AutonConfigurationConstants.kIsBlueAlience ? Rotation2d.fromDegrees(-40) : Rotation2d.fromDegrees(-35)

    AutonConfigurationConstants.robotPositions.put("LeftNoteShootPose", new MirrablePose2d(new Pose2d(1.85 + .06, 7 + .03, Rotation2d.fromDegrees(37.5))));
    AutonConfigurationConstants.robotPositions.put("CenterNoteShootPose", new MirrablePose2d(new Pose2d(2.23 + .08, 5.55, new Rotation2d(0))));
    AutonConfigurationConstants.robotPositions.put("RightNoteShootPose", new MirrablePose2d(new Pose2d(1.85, 4.11,  Rotation2d.fromDegrees(-37.5))));

    AutonConfigurationConstants.robotPositions.put("CenterNoteShootPoseFar", new MirrablePose2d(new Pose2d(2.27, 5.55, new Rotation2d(0))));


    AutonConfigurationConstants.robotPositions.put("LeftNoteIntakePose", new MirrablePose2d(new Pose2d(2.6, 7.00, new Rotation2d(0))));
    AutonConfigurationConstants.robotPositions.put("CenterNoteIntakePose", new MirrablePose2d(new Pose2d(2.6, 5.55, new Rotation2d(0))));
    AutonConfigurationConstants.robotPositions.put("RightNoteIntakePose", new MirrablePose2d(new Pose2d(2.5, 4.11, new Rotation2d(0))));

    AutonConfigurationConstants.robotPositions.put("LeftNoteIntakeZero", new MirrablePose2d(new Pose2d(1.7, 7.00, new Rotation2d(0))));
    AutonConfigurationConstants.robotPositions.put("RightNoteIntakeZero", new MirrablePose2d(new Pose2d(2, 4.11, new Rotation2d(0))));

    AutonConfigurationConstants.robotPositions.put("LeftNoteShootPoseInside", new MirrablePose2d(new Pose2d(2.2, 6.2, Rotation2d.fromDegrees(20))));
    AutonConfigurationConstants.robotPositions.put("FarLeftNoteIntakePose", new MirrablePose2d(new Pose2d(7.93, 5.5116, new Rotation2d(0))));
    AutonConfigurationConstants.robotPositions.put("FarFarLeftNoteIntakePose", new MirrablePose2d(new Pose2d(8.18, 7.188, new Rotation2d(0))));// 7.188, new Rotation2d(0))));
    AutonConfigurationConstants.robotPositions.put("FarFarLeftNoteIntakePoseLeft", new MirrablePose2d(new Pose2d(7.93 + .25, 7.48, new Rotation2d(0))));// 7.188, new Rotation2d(0))));

    //AutonConfigurationConstants.robotPositions.put("FarLeftNoteIntakePose", new MirrablePose2d(new Pose2d(7.8, 5.5116, new Rotation2d(0))));
    AutonConfigurationConstants.robotPositions.put("LeftNoteNextToNoteShootPose", new MirrablePose2d(new Pose2d(2.31 +.02, 6.7+.02, Rotation2d.fromDegrees(25))));// 7.188, new Rotation2d(0))));


    //AutonConfigurationConstatns.robotPositions.put("CenterNoteLeavePose", new MirrablePose2d(new Pose2d(2.20, 5.55, new Rotation2d(0)), !AutonConfigurationConstatns.kIsBlueAlience));
    //AutonConfigurationConstatns.robotPositions.put("RightNoteLeavePose", new MirrablePose2d(new Pose2d(2.20, 4.15, new Rotation2d(0)), !AutonConfigurationConstatns.kIsBlueAlience));
  }

  /*
   * Given the name of an auton, populates the corresponding autonomous object in AutonConfigurationConstants
   */
  public Command configureAutons(String autonName) {
    switch (autonName.split(":")[0]) {
      case "Left":
        MirrablePose2d startingPoseLeft = AutonConfigurationConstants.kLeftStartingPose;
        Pose2d startingPose = startingPoseLeft.getPose(!AutonConfigurationConstants.kIsBlueAlliance);
        this.m_driveSubsystem.setRobotPose(new Pose2d(startingPose.getTranslation(), this.m_driveSubsystem.getHeadingInRotation2d()));
        break;
      
      case "Center":
          MirrablePose2d startingPoseCenter = AutonConfigurationConstants.kCenterStartingPose;
          this.m_driveSubsystem.setRobotPose(startingPoseCenter.getPose(!AutonConfigurationConstants.kIsBlueAlliance));
        break;
      
      case "Right": 
          MirrablePose2d startingPoseRight = AutonConfigurationConstants.kRightStartingPose;
          this.m_driveSubsystem.setRobotPose(startingPoseRight.getPose(!AutonConfigurationConstants.kIsBlueAlliance));
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
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter.add(new WaitCommandWrapper(1.2));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeft.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeft.add(new StopShooterCommand(m_shooterSubsystem));
  
      return GenerateAuto.generateAuto(autonName, AutonConfigurationConstants.kLeft_ShootPreloadedLeft);
    
    case "Left:ShootPreloadedLeftCenter":
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter.add(new RampShooterAtDifforentSpeedCommand(m_shooterSubsystem));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter.add(createGoToPositionCommand("LeftNoteShootPose"));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter.add(createIntakeCommand("LeftNoteIntakePose", AutonConfigurationConstants.kLeftNoteIntakeDownTime));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter.add(createGoToPositionCommand("CenterNoteShootPose"));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter.add(createIntakeCommand("CenterNoteIntakePose", AutonConfigurationConstants.kCenterNoteIntakeDownTime + .3));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter.add(createGoToPositionCommand("CenterNoteShootPose"));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter.add(new WaitCommandWrapper(1.2));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));

      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter.add(new StopShooterCommand(m_shooterSubsystem));

      return GenerateAuto.generateAuto(autonName, AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter);

    case "Left:ShootPreloadedLeftCenterFarFarLeft":
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterFarFarLeft.add(new RampShooterAtDifforentSpeedCommand(m_shooterSubsystem));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterFarFarLeft.add(createGoToPositionCommand("LeftNoteShootPose"));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterFarFarLeft.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterFarFarLeft.add(createIntakeCommand("LeftNoteIntakePose", AutonConfigurationConstants.kLeftNoteIntakeDownTime));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterFarFarLeft.add(createGoToPositionCommand("CenterNoteShootPose"));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterFarFarLeft.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterFarFarLeft.add(createIntakeCommand("CenterNoteIntakePose", AutonConfigurationConstants.kCenterNoteIntakeDownTime + .3));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterFarFarLeft.add(createGoToPositionCommand("CenterNoteShootPose"));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterFarFarLeft.add(new WaitCommandWrapper(1.2));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterFarFarLeft.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterFarFarLeft.add(disableVision());
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterFarFarLeft.add(createIntakeCommandCenter("FarFarLeftNoteIntakePose", AutonConfigurationConstants.kLeftNoteIntakeDownTime));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterFarFarLeft.add(new StopShooterCommand(m_shooterSubsystem));

      return GenerateAuto.generateAuto(autonName, AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterFarFarLeft);
    
    case "Left:ShootPreloadedLeftCenterRight":
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter.add(new RampShooterAtDifforentSpeedCommand(m_shooterSubsystem));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter.add(createGoToPositionCommand("LeftNoteShootPose"));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter.add(createIntakeCommand("LeftNoteIntakePose", AutonConfigurationConstants.kLeftNoteIntakeDownTime));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter.add(createGoToPositionCommand("CenterNoteShootPose"));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter.add(createIntakeCommand("CenterNoteIntakePose", AutonConfigurationConstants.kCenterNoteIntakeDownTime));
      //AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterRight.add(disableVision());
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterRight.add(createGoToPositionCommand("RightNoteShootPose"));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterRight.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterRight.add(createGoToPositionCommand("RightNoteIntakeZero"));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterRight.add(createIntakeCommand("RightNoteIntakePose", AutonConfigurationConstants.kRightNoteIntakeDownTime));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterRight.add(createGoToPositionCommand("RightNoteShootPose"));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenter.add(new WaitCommandWrapper(1.2));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterRight.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
      AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterRight.add(new StopShooterCommand(m_shooterSubsystem));

      return GenerateAuto.generateAuto(autonName, AutonConfigurationConstants.kLeft_ShootPreloadedLeftCenterRight);

    case "Center:ShootPreloaded":
      AutonConfigurationConstants.kCenter_ShootPreloaded.add(new RampShooterAtDifforentSpeedCommand(m_shooterSubsystem));
      AutonConfigurationConstants.kCenter_ShootPreloaded.add(createGoToPositionCommand("CenterNoteShootPose"));
      AutonConfigurationConstants.kCenter_ShootPreloaded.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));

      return GenerateAuto.generateAuto(autonName, AutonConfigurationConstants.kCenter_ShootPreloaded);

      case "Right:ShootPreloaded":
        AutonConfigurationConstants.kRight_ShootPreloaded.add(new RampShooterAtDifforentSpeedCommand(m_shooterSubsystem));
        AutonConfigurationConstants.kRight_ShootPreloaded.add(createGoToPositionCommand("RightNoteShootPose"));
        AutonConfigurationConstants.kRight_ShootPreloaded.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));

        return GenerateAuto.generateAuto(autonName, AutonConfigurationConstants.kRight_ShootPreloaded);
      
        case "Right:ShootPreloadedRight":
        AutonConfigurationConstants.kRight_ShootPreloadedRight.add(new RampShooterAtDifforentSpeedCommand(m_shooterSubsystem));
        AutonConfigurationConstants.kRight_ShootPreloadedRight.add(createGoToPositionCommand("RightNoteShootPose"));
        AutonConfigurationConstants.kRight_ShootPreloadedRight.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
        AutonConfigurationConstants.kRight_ShootPreloadedRight.add(createGoToPositionCommand("RightNoteIntakeZero"));
        AutonConfigurationConstants.kRight_ShootPreloadedRight.add(createIntakeCommand("RightNoteIntakePose", AutonConfigurationConstants.kLeftNoteIntakeDownTime));
        AutonConfigurationConstants.kRight_ShootPreloadedRight.add(createGoToPositionCommand("RightNoteShootPose"));
        AutonConfigurationConstants.kRight_ShootPreloadedRight.add(new WaitCommandWrapper(1.2));
        AutonConfigurationConstants.kRight_ShootPreloadedRight.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
        AutonConfigurationConstants.kRight_ShootPreloadedRight.add(new StopShooterCommand(m_shooterSubsystem));

        return GenerateAuto.generateAuto(autonName, AutonConfigurationConstants.kRight_ShootPreloadedRight);

      case "Right:ShootPreloadedRightCenter":
        AutonConfigurationConstants.kRight_ShootPreloadedRightCenter.add(new RampShooterAtDifforentSpeedCommand(m_shooterSubsystem));
        AutonConfigurationConstants.kRight_ShootPreloadedRightCenter.add(createGoToPositionCommand("RightNoteShootPose"));
        AutonConfigurationConstants.kRight_ShootPreloadedRightCenter.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
        AutonConfigurationConstants.kRight_ShootPreloadedRightCenter.add(createGoToPositionCommand("RightNoteIntakeZero"));
        AutonConfigurationConstants.kRight_ShootPreloadedRightCenter.add(createIntakeCommand("RightNoteIntakePose", AutonConfigurationConstants.kLeftNoteIntakeDownTime));
        //AutonConfigurationConstants.kRight_ShootPreloadedRightCenter.add(disableVision());
        AutonConfigurationConstants.kRight_ShootPreloadedRightCenter.add(createGoToPositionCommand("CenterNoteShootPose"));
        AutonConfigurationConstants.kRight_ShootPreloadedRightCenter.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
        AutonConfigurationConstants.kRight_ShootPreloadedRightCenter.add(createIntakeCommand("CenterNoteIntakePose", AutonConfigurationConstants.kCenterNoteIntakeDownTime));
        AutonConfigurationConstants.kRight_ShootPreloadedRightCenter.add(createGoToPositionCommand("CenterNoteShootPose"));
        AutonConfigurationConstants.kRight_ShootPreloadedRightCenter.add(new WaitCommandWrapper(1.5));
        AutonConfigurationConstants.kRight_ShootPreloadedRightCenter.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
        AutonConfigurationConstants.kRight_ShootPreloadedRightCenter.add(new StopShooterCommand(m_shooterSubsystem));

        return GenerateAuto.generateAuto(autonName, AutonConfigurationConstants.kRight_ShootPreloadedRightCenter);
      
      case "Left:ShootPreloadedFarFarLeft":
        AutonConfigurationConstants.kLeft_ShootPreloadedFarFarLeft.add(new RampShooterAtDifforentSpeedCommand(m_shooterSubsystem));
        AutonConfigurationConstants.kLeft_ShootPreloadedFarFarLeft.add(createGoToPositionCommand("LeftNoteShootPoseInside"));
        AutonConfigurationConstants.kLeft_ShootPreloadedFarFarLeft.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
        AutonConfigurationConstants.kLeft_ShootPreloadedFarFarLeft.add(disableVision());
        AutonConfigurationConstants.kLeft_ShootPreloadedFarFarLeft.add(createIntakeCommand("FarFarLeftNoteIntakePose", 0));
        AutonConfigurationConstants.kLeft_ShootPreloadedFarFarLeft.add(createGoToPositionCommand("LeftNoteShootPoseInside"));
        AutonConfigurationConstants.kLeft_ShootPreloadedFarFarLeft.add(enableVision());
        AutonConfigurationConstants.kLeft_ShootPreloadedFarFarLeft.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
        AutonConfigurationConstants.kLeft_ShootPreloadedFarFarLeft.add(new StopShooterCommand(m_shooterSubsystem));

        return GenerateAuto.generateAuto(autonName, AutonConfigurationConstants.kLeft_ShootPreloadedFarFarLeft);
     
      case "Left:ShootPreloadedFarFarLeftFarLeft":   
          
        AutonConfigurationConstants.kLeft_ShootPreloadedFarFarLeftFarLeft.add(new RampShooterAtDifferentSpeedAutonCommand(m_shooterSubsystem));

        AutonConfigurationConstants.kLeft_ShootPreloadedFarFarLeftFarLeft.add(createGoToPositionCommand("LeftNoteNextToNoteShootPose"));
        AutonConfigurationConstants.kLeft_ShootPreloadedFarFarLeftFarLeft.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
        AutonConfigurationConstants.kLeft_ShootPreloadedFarFarLeftFarLeft.add(disableVision());

        AutonConfigurationConstants.kLeft_ShootPreloadedFarFarLeftFarLeft.add(new PickupFromGroundCommand(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem, m_ledSubsystem));//createIntakeCommand("LeftNoteNextToNoteShootPose", .5));//PickupFromGroundCommand(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem, m_ledSubsystem));//createIntakeCommand("LeftNoteNextToNoteShootPose", AutonConfigurationConstants.kLeftNoteIntakeDownTime));
        //AutonConfigurationConstants.kLeft_ShootPreloadedFarFarLeftFarLeft.add(createGoToPositionCommand("LeftNoteNextToNoteShootPose"));
        AutonConfigurationConstants.kLeft_ShootPreloadedFarFarLeftFarLeft.add(new WaitCommandWrapper(1.2));

        //AutonConfigurationConstants.kLeft_ShootPreloadedFarFarLeftFarLeft.add(enableVision());
        //AutonConfigurationConstants.kLeft_ShootPreloadedFarFarLeftFarLeft.add(createGoToPositionCommand("LeftNoteShootPoseInside"));
        AutonConfigurationConstants.kLeft_ShootPreloadedFarFarLeftFarLeft.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
        AutonConfigurationConstants.kLeft_ShootPreloadedFarFarLeftFarLeft.add(createIntakeCommand("FarFarLeftNoteIntakePoseLeft", 0));
        AutonConfigurationConstants.kLeft_ShootPreloadedFarFarLeftFarLeft.add(enableVision());
        AutonConfigurationConstants.kLeft_ShootPreloadedFarFarLeftFarLeft.add(createGoToPositionCommand("LeftNoteNextToNoteShootPose"));
        AutonConfigurationConstants.kLeft_ShootPreloadedFarFarLeftFarLeft.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
  
        return GenerateAuto.generateAuto(autonName, AutonConfigurationConstants.kLeft_ShootPreloadedFarFarLeftFarLeft);

      case "Center:ShootPreloadedCenter":
        AutonConfigurationConstants.kCenter_ShootPreloadedCenter.add(disableVision());
        AutonConfigurationConstants.kCenter_ShootPreloadedCenter.add(new RampShooterAtDifforentSpeedCommand(m_shooterSubsystem));
        AutonConfigurationConstants.kCenter_ShootPreloadedCenter.add(createGoToPositionCommand("CenterNoteShootPose"));
        AutonConfigurationConstants.kCenter_ShootPreloadedCenter.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
        AutonConfigurationConstants.kCenter_ShootPreloadedCenter.add(createIntakeCommand("CenterNoteIntakePose", AutonConfigurationConstants.kLeftNoteIntakeDownTime));
        AutonConfigurationConstants.kCenter_ShootPreloadedCenter.add(new WaitCommandWrapper(1.2));
        AutonConfigurationConstants.kCenter_ShootPreloadedCenter.add(new StopShooterCommand(m_shooterSubsystem));
        

        return GenerateAuto.generateAuto(autonName, AutonConfigurationConstants.kCenter_ShootPreloadedCenter);

      case "Center:ShootPreloadedCenterFarFarLeft":
        AutonConfigurationConstants.kCenter_ShootPreloadedCenterFarFarLeft.add(new RampShooterAtDifforentSpeedCommand(m_shooterSubsystem));
        AutonConfigurationConstants.kCenter_ShootPreloadedCenterFarFarLeft.add(disableVision());
        AutonConfigurationConstants.kCenter_ShootPreloadedCenterFarFarLeft.add(createGoToPositionCommandCenter("CenterNoteShootPoseFar"));
        AutonConfigurationConstants.kCenter_ShootPreloadedCenterFarFarLeft.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
        AutonConfigurationConstants.kCenter_ShootPreloadedCenterFarFarLeft.add(createIntakeCommandCenter("CenterNoteIntakePose", AutonConfigurationConstants.kLeftNoteIntakeDownTime));
        AutonConfigurationConstants.kCenter_ShootPreloadedCenterFarFarLeft.add(new WaitCommandWrapper(1.2));
        AutonConfigurationConstants.kCenter_ShootPreloadedCenterFarFarLeft.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
        AutonConfigurationConstants.kCenter_ShootPreloadedCenterFarFarLeft.add(createIntakeCommandCenter("FarFarLeftNoteIntakePose", AutonConfigurationConstants.kLeftNoteIntakeDownTime));
        AutonConfigurationConstants.kCenter_ShootPreloadedCenterFarFarLeft.add(enableVision());
        AutonConfigurationConstants.kCenter_ShootPreloadedCenterFarFarLeft.add(createGoToPositionCommandCenter("CenterNoteShootPose"));
        AutonConfigurationConstants.kCenter_ShootPreloadedCenterFarFarLeft.add(new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem));
        //AutonConfigurationConstants.kCenter_ShootPreloadedCenterFarFarLeft.add(new StopShooterCommand(m_shooterSubsystem));

        return GenerateAuto.generateAuto(autonName, AutonConfigurationConstants.kCenter_ShootPreloadedCenterFarFarLeft);
      }

    return null;

  }

  public  void displayGreenWhenAtAngle() {
    boolean degreesEqual = Math.abs( this.m_driveSubsystem.getHeadingInRotation2d().getDegrees() - AutonConfigurationConstants.robotPositions.get("LeftNoteNextToNoteShootPose").
     getPose(!AutonConfigurationConstants.kIsBlueAlliance).getRotation().getDegrees()) <= 1;
     SmartDashboard.putBoolean("Degree Equal", degreesEqual);
    if(degreesEqual && AutonConfigurationConstants.kInAuto) {
      Commands.runOnce(this::setLEDColor, this.m_ledSubsystem);
    }
  }

  public void setLEDColor() {
    this.m_ledSubsystem.setSolidColor(new int[] {255, 0, 0});
  }

  private Command stop() {
    return Commands.runOnce(this::stopUpChain);
  }

  private boolean stopUpChain() {
      return AutonConfigurationConstants.kSTOP = isRobotCloserThanPoint() ? true : false;
  }

  private boolean isRobotCloserThanPoint() {
    return this.m_driveSubsystem.getRobotPose().getTranslation().getX() < 3;
  }
  private Command createGoToPoseUntillCloseThanReschedule() {
          return new SequentialGroupCommand(
            new Command[] {new RunCommandUtillConditionCommand(createGoToPositionCommand("LeftNoteShootPose"), this::isRobotCloserThanPoint),
            createGoToPositionCommand("LeftNoteShootPose")
        });

  }

  /*
   * Given the name of a target field position and a time to wait before doing so, create a command that runs the intake and moves to the position
   */
  public Command createIntakeCommand(String poseName, double waitTime) {
    return new ParallelRaceGroupCommand(new PickupFromGroundCommand(m_intakeSubsystem, m_pivotSubsystem,
     m_intakeSensorSubsystem, this.m_ledSubsystem),
      new GoToPositionAfterTimeWithPIDS(
        new GoToPositionWithPIDSAuto(m_driveSubsystem, 
        AutonConfigurationConstants.robotPositions.get(poseName).getPose(!AutonConfigurationConstants.kIsBlueAlliance)), waitTime));
  }

  public Command createIntakeCommandCenter(String poseName, double waitTime) {
    return new ParallelRaceGroupCommand(new PickupFromGroundCommand(m_intakeSubsystem, m_pivotSubsystem,
     m_intakeSensorSubsystem, this.m_ledSubsystem),
      new GoToPositionAfterTimeWithPIDSCenter(
        new GoToPositionWithPIDSAutoCenter(m_driveSubsystem, 
        AutonConfigurationConstants.robotPositions.get(poseName).getPose(!AutonConfigurationConstants.kIsBlueAlliance)), waitTime));

  }

  /*
   * Given the name of a target field position, create a command that moves to the position
   */
  public GoToPositionWithPIDSAuto createGoToPositionCommand(String poseName) {
    return new GoToPositionWithPIDSAuto(this.m_driveSubsystem,
          AutonConfigurationConstants.robotPositions.get(poseName).getPose(!AutonConfigurationConstants.kIsBlueAlliance));
  }

  public GoToPositionWithPIDSAutoCenter createGoToPositionCommandCenter(String poseName) {
    return new GoToPositionWithPIDSAutoCenter(this.m_driveSubsystem,
          AutonConfigurationConstants.robotPositions.get(poseName).getPose(!AutonConfigurationConstants.kIsBlueAlliance));
  }


  /*
   * Given the name of a target field position and a percentage at which to start shooting, create a command that moves to a position and shoots during transit
   */
   public GoToPoseAutonWhileShootingWithPIDS createGoToPoseAutonWhileShooting(String poseName, double percentToPose) {
    return new GoToPoseAutonWhileShootingWithPIDS(
          this.m_driveSubsystem,
          new HandOffToShooterAuton(m_intakeSubsystem, m_pivotSubsystem, m_intakeSensorSubsystem),
          AutonConfigurationConstants.robotPositions.get(poseName).getPose(!AutonConfigurationConstants.kIsBlueAlliance),
          percentToPose
    );    
  }

public Command enableVision() {
    return Commands.runOnce(() -> {VisionConstants.useVision = true;});
  }

  public Command disableVision() {
    return Commands.runOnce(() -> {VisionConstants.useVision = false;});
  }

  public void onAllianceChanged(Optional<Alliance> currentAlliance) {
    this.m_driveSubsystem.getPoseEstimatorSubsystem().setAlliance(currentAlliance);
  }


}