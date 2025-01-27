package frc.robot.subsystems.SwerveDrive;

import java.util.List;

//import org.photonvision.PhotonCamera;
//import org.photonvision.PhotonPoseEstimator;

import com.ctre.phoenix6.hardware.Pigeon2;
//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
//import com.pathplanner.lib.util.ReplanningConfig;

//import edu.wpi.first.math.Matrix;
//import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.math.numbers.N1;
//import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
//import edu.wpi.first.wpilibj.smartdashboard.Field2d;
//SmartDashBoard.Field2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.WPILIBTrajectoryConstants;
import frc.robot.DeprecatedSystems.PoseEstimatorSubsystem;
//import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
//import frc.robot.subsystems.Vision.PhotonVisionPoseEstimator;

/*
 * The subsystem that controls our swerve drive
 */
public class DriveSubsystem extends SubsystemBase {

    private ChassisSpeeds desiredChassisSpeeds;

    private final SwerveModule frontLeftSwerveModule;
    private final SwerveModule frontRightSwerveModule;
    private final SwerveModule backLeftSwerveModule;
    private final SwerveModule backRightSwerveModule;

    //private int _updateCount;

    //private static final Matrix<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, 0.1);//gray matter has lower

  /**
   * Standard deviations of the vision measurements. Increase these numbers to
   * trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
   * radians.
   */
  //private static final Matrix<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, 0.9); // gray mater has higher


    private Pigeon2 m_gyro;

    private final SwerveDrivePoseEstimator poseEstimator;
    private final Pose2d startingPose = new Pose2d(0, 0, new Rotation2d(0));

    //private final Field2d field2d = new Field2d();

    //private Matrix<N3, N1> visionMeasurementStdDevs = new Matrix<N3, N1>(Nat.N3(), Nat.N1(), new double[] {0.5, 0.5, 0.9});

    private PoseEstimatorSubsystem m_poseEstimatorSubsystem;
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

        //NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight-zeta");

        this.m_poseEstimatorSubsystem = new PoseEstimatorSubsystem(this);//PhotonVisionPoseEstimator(this);
        //ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");
        //visionTab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
        //visionTab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
        
      }
      
        //SmartDashBoard.updateValues(); 
        //this.m_gyro.getYaw().setUpdateFrequency(100);
        //this.m_gyro.optimizeBusUtilization()

  @Override
  public void periodic() {
    //updateOdometry();

    if (desiredChassisSpeeds != null) {  
          SwerveModuleState[] desiredStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(desiredChassisSpeeds);   
      // If we're not trying to move, we lock the angles of the wheels
      if (desiredChassisSpeeds.vxMetersPerSecond == 0.0 && desiredChassisSpeeds.vyMetersPerSecond == 0.0
          && desiredChassisSpeeds.omegaRadiansPerSecond == 0.0) {
        SwerveModuleState[] currentStates = getModuleStates();

        //TODO: We may want to just call this on all four states individually, the for loop may introduce slight overhead
        for(int i = 0; i < currentStates.length; i++) {
            desiredStates[i].angle = currentStates[i].angle;
        }
      }

      // Positive angles should be counter clockwise.
      setModuleStates(desiredStates);
    }
    // Resets the desiredChassisSpeeds to null to stop it from "sticking" to the last states
    desiredChassisSpeeds = null;
    //updateDashboard();
  }

  /*
   * Use our pose estimator to update our odometry measurements
   */
 

  /*
   * Posts helpful debugging info to //SmartDashBoard
   */
  /*private void updateDashboard() {
    //if(_updateCount++ >= 0)
    //{
     // _updateCount = 0;

      //SmartDashboard.putNumber("FL MPS", Math.abs(this.frontLeftSwerveModule.getDriveMotorSpeedInMetersPerSecond()));
      //SmartDashboard.putNumber("FL Angle", this.frontLeftSwerveModule.getTurningEncoderAngleDegrees().getDegrees());
      //SmartDashBoard.putNumber("FL Angle From Swerve Module Position", this.frontLeftSwerveModule.getPosition().angle.getDegrees());
      //SmartDashBoard.putNumber("FL Distence", this.frontLeftSwerveModule.getDistance());
      //SmartDashBoard.putNumber("FL Distence In Meters From SwerveModule Position", this.frontLeftSwerveModule.getPosition().distanceMeters);

      //SmartDashBoard.putNumber("FR MPS", this.frontRightSwerveModule.getDriveMotorSpeedInMetersPerSecond());
      //SmartDashBoard.putNumber("FR Angle", this.frontRightSwerveModule.getTurningEncoderAngleDegrees().getDegrees());

      //SmartDashBoard.putNumber("BL MPS", this.backLeftSwerveModule.getDriveMotorSpeedInMetersPerSecond());
      //SmartDashBoard.putNumber("BL Angle", this.backLeftSwerveModule.getTurningEncoderAngleDegrees().getDegrees());

      //SmartDashBoard.putNumber("BR MPS", this.backRightSwerveModule.getDriveMotorSpeedInMetersPerSecond());
      //SmartDashBoard.putNumber("BR Angle", this.backRightSwerveModule.getTurningEncoderAngleDegrees().getDegrees());

      //SmartDashBoard.putNumber("Robot Heading in Degrees", this.m_gyro.getAngle()); 
     // }
    }
  */

   /*
    * Returns an array containing the positions of all swerve modules
    */
  public SwerveModulePosition[] getModulePositions() {
    //TODO: We don't really need to set this array to a variable before returning it, even though it *probably* doesn't affect performance much
    SwerveModulePosition[] positions = {
        this.frontLeftSwerveModule.getPosition(),
        this.frontRightSwerveModule.getPosition(),
        this.backLeftSwerveModule.getPosition(),
        this.backRightSwerveModule.getPosition()
    };

    return positions;
  }
  /*
   * Returns an array containing the states of all swerve modules
   */
  private SwerveModuleState[] getModuleStates() {
    //TODO: See getModulePosition's todo
    SwerveModuleState[] swerveModuleStates = {
        this.frontLeftSwerveModule.getState(),
        this.frontRightSwerveModule.getState(),
        this.backLeftSwerveModule.getState(),
        this.backRightSwerveModule.getState()
      };

    return swerveModuleStates;
  }
  
  /*
   * Sets the desired states of the swerves to the given states.
   * This just changes the targets of our PID loop.
   * Make sure to pass them in as follows:
   * Front left, front right, back left, back right
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates, SwerveDriveConstants.kMaxSpeedMetersPerSecond);
    this.frontLeftSwerveModule.setDesiredState(desiredStates[0]);
    this.frontRightSwerveModule.setDesiredState(desiredStates[1]);
    this.backLeftSwerveModule.setDesiredState(desiredStates[2]);
    this.backRightSwerveModule.setDesiredState(desiredStates[3]); 

    //SmartDashBoard.putNumber("FL Desired MPS", desiredStates[0].speedMetersPerSecond);
    /* 
    //SmartDashBoard.putNumber("FL Desired Angle", desiredStates[0].angle.getDegrees());

    
    //SmartDashBoard.putNumber("FR Desired MPS", desiredStates[1].speedMetersPerSecond);
    //SmartDashBoard.putNumber("FR Desired Angle", desiredStates[1].angle.getDegrees());

    //SmartDashBoard.putNumber("BL Desired MPS", desiredStates[2].speedMetersPerSecond);
    //SmartDashBoard.putNumber("BL Desired Angle", desiredStates[2].angle.getDegrees());

    //SmartDashBoard.putNumber("BR Desired MPS", desiredStates[3].speedMetersPerSecond);
    //SmartDashBoard.putNumber("BR Desired Angle", desiredStates[3].angle.getDegrees());
    */
  }

  /*
   * Returns the module states in the form of chassis speeds
   */
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

  /*
   * Returns the current chassis speeds in the form of a linear velocity
   */
  public double getCurrentChassisSpeeds()
  {
    ChassisSpeeds currentSpeeds = SwerveDriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    //TODO: Ew sqrt bad (I see it's the pythagorean theorem so probably not fixable)
    double linearVeloicity = Math.sqrt((currentSpeeds.vxMetersPerSecond * currentSpeeds.vxMetersPerSecond) * (currentSpeeds.vyMetersPerSecond * currentSpeeds.vyMetersPerSecond));
    return linearVeloicity;
  }

  public Rotation2d getCurrentChassisHeading()
  {
    ChassisSpeeds currentSpeeds = SwerveDriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    //TODO: Ew atan2 bad. Is there no more efficient way to get the heading?
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
    //TODO: We don't need to be constructing new locking states every time we lock the swerves. Make them into constants or something.
    SwerveModuleState m_frontLeftLockupState = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
    SwerveModuleState m_frontRightLockupState = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    SwerveModuleState m_rearLeftLockupState = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    SwerveModuleState m_rearRightLockupState = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
    this.frontLeftSwerveModule.setDesiredState(m_frontLeftLockupState);
    this.frontRightSwerveModule.setDesiredState(m_frontRightLockupState);
    this.backLeftSwerveModule.setDesiredState(m_rearLeftLockupState);
    this.backRightSwerveModule.setDesiredState(m_rearRightLockupState);
  }

  /* 
  private String getFomattedPose() {
    Pose2d pose = this.poseEstimator.getEstimatedPosition();
    return String.format("(%.2f, %.2f) %.2f degrees",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }
  */

  public Pose2d getRobotPose() {
    return this.m_poseEstimatorSubsystem.getCurrentPose();// this.poseEstimator.getEstimatedPosition();
  }

  public void setRobotPose(Pose2d newPose) {
    this.m_poseEstimatorSubsystem.setCurrentPose(newPose);//this.poseEstimator.resetPosition(this.m_gyro.getRotation2d(), getModulePositions(), newPose);
  }

  public void resetRobotPose() {
    this.m_gyro.reset();
    this.m_poseEstimatorSubsystem.resetPose();
    //this.m_gyro.reset();
    //Pose2d newPose = new Pose2d();
    //this.poseEstimator.resetPosition(newPose.getRotation(), getModulePositions(), newPose);
  }

  public PoseEstimatorSubsystem getPoseEstimatorSubsystem() {
    //TODO: Inherently inefficient (brought to you by the Anti Getter/Setter Initiative)
    // (this is a joke)
    return m_poseEstimatorSubsystem;
  }



  public void resetRobotHeading() {
    Pose2d estimatedPose = this.poseEstimator.getEstimatedPosition();
    //TODO: We are just making a new pose named estimatedPose that's already a Pose2D. We don't need to be constructing a new Pose2d using the same values!
    //TODO: (cont'd) Since we have to reset rotation, just set rotation to a new Rotation2d on the estimatedPose variable, then pass it in by itself.
    this.poseEstimator.resetPosition(m_gyro.getRotation2d(), getModulePositions(), new Pose2d(estimatedPose.getX(), estimatedPose.getY(), new Rotation2d()));
  }

  public SequentialCommandGroup generateOnTheFlyPath(Pose2d endPose) {
        // Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                getRobotPose(),
                List.of(),
                endPose,
                WPILIBTrajectoryConstants.trajectoryConfig);

      
        // Should not need this our odometry is -64 bit to +64bit
        //thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                this::getRobotPose,
                SwerveDriveConstants.kDriveKinematics,
                WPILIBTrajectoryConstants.kXController,
                WPILIBTrajectoryConstants.kYController,
                WPILIBTrajectoryConstants.kThetaController,
                this::setModuleStates,
                this);

      return new SequentialCommandGroup(
                swerveControllerCommand,
                new InstantCommand(() -> stop()));
  }
}
