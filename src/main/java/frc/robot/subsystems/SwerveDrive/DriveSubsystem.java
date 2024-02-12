package frc.robot.subsystems.SwerveDrive;

import java.util.List;
import java.util.Optional;


import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OnTheFlyGenerationConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.utils.InTeleop;
import frc.robot.utils.LimelightUtil;
import frc.robot.utils.VisionPose;

public class DriveSubsystem extends SubsystemBase {

    private ChassisSpeeds desiredChassisSpeeds;

    private final SwerveModule frontLeftSwerveModule;
    private final SwerveModule frontRightSwerveModule;
    private final SwerveModule backLeftSwerveModule;
    private final SwerveModule backRightSwerveModule;

    private int _updateCount;


    private Pigeon2 m_gyro;
    //private Pose2d startingPosition = new Pose2d(0, 0, new Rotation2d(0));

    private final SwerveDrivePoseEstimator poseEstimator;
    private final Pose2d startingPose = new Pose2d(0, 0, new Rotation2d(0));

    private final Field2d field2d = new Field2d();

    private final ProfiledPIDController headingPIDController;

    public DriveSubsystem() {
        this.frontLeftSwerveModule =  new SwerveModule(
            SwerveDriveConstants.kFrontLeftDriveMotorId, SwerveDriveConstants.kFrontLeftTurnMotorId, SwerveDriveConstants.kFrontLeftTurnEncoderId,
             SwerveDriveConstants.kFrontLeftTurnEncoderOffset, SwerveDriveConstants.kFrontLeftTurnMagnetOffset, SwerveDriveConstants.kFrontLeftDriveMotorRev, SwerveDriveConstants.kFrontLeftTurnMotorRev);
        
        this.frontRightSwerveModule = new SwerveModule(
            SwerveDriveConstants.kFrontRightDriveMotorId, SwerveDriveConstants.kFrontRightTurnMotorId, SwerveDriveConstants.kFrontRightTurnEncoderId,
             SwerveDriveConstants.kFrontRightTurnEncoderOffset, SwerveDriveConstants.kFrontRightTurnMagnetOffset, SwerveDriveConstants.kFrontRightDriveMotorRev, SwerveDriveConstants.kFrontRightTurnMotorRev);

        this.backLeftSwerveModule = new SwerveModule(
            SwerveDriveConstants.kBackLeftDriveMotorId, SwerveDriveConstants.kBackLeftTurnMotorId, SwerveDriveConstants.kBackLeftTurnEncoderId,
             SwerveDriveConstants.kBackLeftTurnEncoderOffset, SwerveDriveConstants.kBackLeftTurnMagnetOffset, SwerveDriveConstants.kBackLeftDriveMotorRev, SwerveDriveConstants.kBackLeftTurnMotorRev);

        this.backRightSwerveModule = new SwerveModule(
            SwerveDriveConstants.kBackRightDriveMotorId, SwerveDriveConstants.kBackRightTurnMotorId, SwerveDriveConstants.kBackRightTurnEncoderId,
             SwerveDriveConstants.kBackRightTurnEncoderOffset, SwerveDriveConstants.kBackRightTurnMagnetOffset, SwerveDriveConstants.kBackRightDriveMotorRev, SwerveDriveConstants.kBackRightTurnMotorRev);

        this.m_gyro = new Pigeon2(SwerveDriveConstants.kGyroId);

        this.m_gyro.reset();  

        this.poseEstimator = new SwerveDrivePoseEstimator(SwerveDriveConstants.kDriveKinematics, this.m_gyro.getRotation2d(), 
        getModulePositions(), startingPose);

        NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight-zeta");


        ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");
        visionTab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
        visionTab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);

         AutoBuilder.configureHolonomic(
                this.poseEstimator::getEstimatedPosition, // Robot pose supplier
                this::setRobotPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        AutoConstants.kTranslationAutoPID, // Translation PID constants
                        AutoConstants.kRotationAutoPID, // Rotation PID constants
                        AutoConstants.kMaxAutonSpeedInMetersPerSecond , // Max module speed, in m/s
                        SwerveDriveConstants.kRadiusFromCenterToFarthestSwerveModule, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig(true, false) // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    Optional<Alliance> alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

        this.headingPIDController = new ProfiledPIDController(SwerveDriveConstants.kHeadingPIDControllerP, SwerveDriveConstants.kHeadingPIDControllerI,
                                    SwerveDriveConstants.kHeadingPIDControllerD, SwerveDriveConstants.kThetaControllerConstraints);
        this.headingPIDController.setTolerance(SwerveDriveConstants.kHeadingPIDControllerTolerance);
        this.headingPIDController.setIntegratorRange(-0.3, 0.3);

        this.headingPIDController.reset(this.poseEstimator.getEstimatedPosition().getRotation().getDegrees());
 
        SmartDashboard.updateValues(); 
    }

  @Override
  public void periodic() {
    updateOdometry();

    if (desiredChassisSpeeds != null) {  
          SwerveModuleState[] desiredStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(desiredChassisSpeeds);   
      // If we're not trying to move, we lock the angles of the wheels
      if (desiredChassisSpeeds.vxMetersPerSecond == 0.0 && desiredChassisSpeeds.vyMetersPerSecond == 0.0
          && desiredChassisSpeeds.omegaRadiansPerSecond == 0.0) {
        SwerveModuleState[] currentStates = getModuleStates();
        for(int i = 0; i < currentStates.length; i++) {
            desiredStates[i].angle = currentStates[i].angle;
        }
      }

      // Positive angles should be counter clockwise.
      setModuleStates(desiredStates);
    }
    // Resets the desiredChassisSpeeds to null to stop it from "sticking" to the last states
    desiredChassisSpeeds = null;
    updateDashboard();
  }

 private void updateOdometry() {
    this.poseEstimator.update(this.m_gyro.getRotation2d(), getModulePositions());
    
    VisionPose estimatedPose = LimelightUtil.getBotpose();
    Pose2d currentEstimatedPose = this.poseEstimator.getEstimatedPosition();

    boolean withInOneMeterX = Math.abs(currentEstimatedPose.getX() - estimatedPose.getPose().getX()) <= 1;

    boolean withInOneMeterY = Math.abs(currentEstimatedPose.getY() - estimatedPose.getPose().getY()) <= 1;

    boolean withInOneMeter = withInOneMeterX && withInOneMeterY;
    if(estimatedPose.isValidTarget() && InTeleop.inTeleop) { //&& withInOneMeter) {
      this.poseEstimator.addVisionMeasurement(estimatedPose.getPose(), estimatedPose.getTimeStamp());
    }
    this.field2d.setRobotPose(this.poseEstimator.getEstimatedPosition());
    
  }

  private void updateDashboard() {
    if(_updateCount++ >= 0)
    {
      _updateCount = 0;

      SmartDashboard.putNumber("FL MPS", Math.abs(this.frontLeftSwerveModule.getDriveMotorSpeedInMetersPerSecond()));
      SmartDashboard.putNumber("FL Angle", this.frontLeftSwerveModule.getTurningEncoderAngleDegrees().getDegrees());
      SmartDashboard.putNumber("FL Angle From Swerve Module Position", this.frontLeftSwerveModule.getPosition().angle.getDegrees());
      SmartDashboard.putNumber("FL Distence", this.frontLeftSwerveModule.getDistance());
      SmartDashboard.putNumber("FL Distence In Meters From SwerveModule Position", this.frontLeftSwerveModule.getPosition().distanceMeters);

      SmartDashboard.putNumber("FR MPS", this.frontRightSwerveModule.getDriveMotorSpeedInMetersPerSecond());
      SmartDashboard.putNumber("FR Angle", this.frontRightSwerveModule.getTurningEncoderAngleDegrees().getDegrees());

      SmartDashboard.putNumber("BL MPS", this.backLeftSwerveModule.getDriveMotorSpeedInMetersPerSecond());
      SmartDashboard.putNumber("BL Angle", this.backLeftSwerveModule.getTurningEncoderAngleDegrees().getDegrees());

      SmartDashboard.putNumber("BR MPS", this.backRightSwerveModule.getDriveMotorSpeedInMetersPerSecond());
      SmartDashboard.putNumber("BR Angle", this.backRightSwerveModule.getTurningEncoderAngleDegrees().getDegrees());

      SmartDashboard.putNumber("Robot Heading in Degrees", this.m_gyro.getAngle()); 
      }
    }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = {
        this.frontLeftSwerveModule.getPosition(),
        this.frontRightSwerveModule.getPosition(),
        this.backLeftSwerveModule.getPosition(),
        this.backRightSwerveModule.getPosition()
    };

    return positions;
  }
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] swerveModuleStates = {
        this.frontLeftSwerveModule.getState(),
        this.frontRightSwerveModule.getState(),
        this.backLeftSwerveModule.getState(),
        this.backRightSwerveModule.getState()
      };

    return swerveModuleStates;
  }
  
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates, SwerveDriveConstants.kMaxSpeedMetersPerSecond);
    this.frontLeftSwerveModule.setDesiredState(desiredStates[0]);
    this.frontRightSwerveModule.setDesiredState(desiredStates[1]);
    this.backLeftSwerveModule.setDesiredState(desiredStates[2]);
    this.backRightSwerveModule.setDesiredState(desiredStates[3]); 

    SmartDashboard.putNumber("FL Desired MPS", desiredStates[0].speedMetersPerSecond);
    /* 
    SmartDashboard.putNumber("FL Desired Angle", desiredStates[0].angle.getDegrees());

    
    SmartDashboard.putNumber("FR Desired MPS", desiredStates[1].speedMetersPerSecond);
    SmartDashboard.putNumber("FR Desired Angle", desiredStates[1].angle.getDegrees());

    SmartDashboard.putNumber("BL Desired MPS", desiredStates[2].speedMetersPerSecond);
    SmartDashboard.putNumber("BL Desired Angle", desiredStates[2].angle.getDegrees());

    SmartDashboard.putNumber("BR Desired MPS", desiredStates[3].speedMetersPerSecond);
    SmartDashboard.putNumber("BR Desired Angle", desiredStates[3].angle.getDegrees());
    */
  }

  public ChassisSpeeds getChassisSpeeds() {
    return SwerveDriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  /* 
  Start of wrapper for the gyro
  */
  public double getTurnRate() {
    return m_gyro.getRate();
  }
/* 
  public double getRoll() {
    return this.m_gyro.getRoll();
  }
  public double getRollRate() {
    double[] xyzDegPerSec = new double[3];
    this.m_gyro.getRawGyro(xyzDegPerSec);
    return xyzDegPerSec[0];
  }

  public double getPitch() {
    return this.m_gyro.getPitch();
  }

  public double getPitchRate() {
    double[] xyzDegPerSec = new double[3];
    this.m_gyro.getRawGyro(xyzDegPerSec);
    return xyzDegPerSec[1];
  }

  public double getYaw() {
    return this.m_gyro.getRoll();
  }

  public double getYawRate() {
    double[] xyzDegPerSec = new double[3];
    this.m_gyro.getRawGyro(xyzDegPerSec);
    return xyzDegPerSec[2];
  }
 */
  public void zeroHeading() {
    this.m_gyro.reset();
  }

  public double getHeading() {
    return this.m_gyro.getRotation2d().getDegrees();
  }

  public Rotation2d getHeadingInRotation2d() {
    return this.m_gyro.getRotation2d();
  }

  public double getCurrentChassisSpeeds()
  {
    ChassisSpeeds currentSpeeds = SwerveDriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    double linearVeloicity = Math.sqrt((currentSpeeds.vxMetersPerSecond * currentSpeeds.vxMetersPerSecond) * (currentSpeeds.vyMetersPerSecond * currentSpeeds.vyMetersPerSecond));
    return linearVeloicity;
  }

  public Rotation2d getCurrentChassisHeading()
  {
    ChassisSpeeds currentSpeeds = SwerveDriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    Rotation2d robotHeading = new Rotation2d(Math.atan2(currentSpeeds.vyMetersPerSecond, currentSpeeds.vxMetersPerSecond));
    return robotHeading;
  }

  /* 
  End of gyro wrapper
  */

  public void drive(ChassisSpeeds chassisSpeeds) {
    this.desiredChassisSpeeds = chassisSpeeds;
  }
  
  public void stop(){
    drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(0,0
        ,0, new Rotation2d(0)));
  }

  public void lockSwerves(){
    // Crank all the swerve turning motors 45 degrees one way or the other
    SwerveModuleState m_frontLeftLockupState = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
    SwerveModuleState m_frontRightLockupState = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    SwerveModuleState m_rearLeftLockupState = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    SwerveModuleState m_rearRightLockupState = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
    this.frontLeftSwerveModule.setDesiredState(m_frontLeftLockupState);
    this.frontRightSwerveModule.setDesiredState(m_frontRightLockupState);
    this.backLeftSwerveModule.setDesiredState(m_rearLeftLockupState);
    this.backRightSwerveModule.setDesiredState(m_rearRightLockupState);
  }

  private String getFomattedPose() {
    Pose2d pose = this.poseEstimator.getEstimatedPosition();
    return String.format("(%.2f, %.2f) %.2f degrees",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  public Pose2d getRobotPose() {
    return this.poseEstimator.getEstimatedPosition();
  }

  public void setRobotPose(Pose2d newPose) {
    this.poseEstimator.resetPosition(this.m_gyro.getRotation2d(), getModulePositions(), newPose);
  }

  public void resetRobotPose() {
    Pose2d newPose = new Pose2d();
    this.poseEstimator.resetPosition(newPose.getRotation(), getModulePositions(), newPose);
  }


  public void resetRobotHeading() {
    this.m_gyro.reset();
    this.poseEstimator.resetPosition(m_gyro.getRotation2d(), getModulePositions(), new Pose2d());
  }

  public SequentialCommandGroup generateOnTheFlyPath(Pose2d endPose) {
        // Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                getRobotPose(),
                List.of(),
                endPose,
                OnTheFlyGenerationConstants.trajectoryConfig);

      
        // Should not need this our odometry is -64 bit to +64bit
        //thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                this::getRobotPose,
                SwerveDriveConstants.kDriveKinematics,
                OnTheFlyGenerationConstants.kXController,
                OnTheFlyGenerationConstants.kYController,
                OnTheFlyGenerationConstants.kThetaController,
                this::setModuleStates,
                this);

      return new SequentialCommandGroup(
                swerveControllerCommand,
                new InstantCommand(() -> stop()));
  }

  public void turnToHeading(double desiredHeading) {
    drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        0,
        0,
        headingPIDController.calculate(poseEstimator.getEstimatedPosition().getRotation().getDegrees(), desiredHeading),
        getRobotPose().getRotation()));
  }

  public void resetHeadingPid(){
    headingPIDController.reset(poseEstimator.getEstimatedPosition().getRotation().getDegrees());
  }

  public boolean isHeadingPidAtGoal() {
    return this.headingPIDController.atGoal();
  }
}
