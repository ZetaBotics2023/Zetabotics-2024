// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveDrive;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkPIDController.AccelStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.utils.SwerveModuleAngleOptimizer;

public class SwerveModule{

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private SparkPIDController drivingPIDController;
  private SparkPIDController turningPIDController;

  private RelativeEncoder driveRelativeEncoder;

  private CANcoder turningAbsoluteEncoder;
  private RelativeEncoder turningRelativeEncoder;

  private double absoluteEncoderOffset;

  public SwerveModule(int driveMotorId, int turningMotorId, int turningAbsoluteEncoderId, double turningEncoderOffset, double turningEncoderMagOffset, boolean driveMotorRev,
   boolean turnMotorRev) {

    this.m_driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    this.m_turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

    this.driveRelativeEncoder = this.m_driveMotor.getEncoder();

    this.turningAbsoluteEncoder = new CANcoder(turningAbsoluteEncoderId);
    this.turningRelativeEncoder = m_turningMotor.getEncoder();

    this.drivingPIDController = this.m_driveMotor.getPIDController();
    this.turningPIDController = this.m_turningMotor.getPIDController();

    this.drivingPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

    this.m_driveMotor.restoreFactoryDefaults();
    this.m_turningMotor.restoreFactoryDefaults();

    this.m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    this.m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
    this.m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);

    this.m_turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    this.m_turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    this.m_turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);

    this.m_driveMotor.setIdleMode(IdleMode.kBrake);
    this.m_turningMotor.setIdleMode(IdleMode.kBrake);

    this.m_driveMotor.setInverted(driveMotorRev);
    this.m_turningMotor.setInverted(turnMotorRev);

    // This may need to be changed
    this.m_driveMotor.setSmartCurrentLimit(40);
    this.m_turningMotor.setSmartCurrentLimit(5);

    //this.turningRelativeEncoder.setVelocityConversionFactor(SwerveModuleConstants.kDriveConversionVelocityFactor);

    this.drivingPIDController.setP(SwerveModuleConstants.kPModuleDriveController);
    this.drivingPIDController.setI(SwerveModuleConstants.kIModuleDriveController);
    this.drivingPIDController.setD(SwerveModuleConstants.kDModuleDriveController);
    this.drivingPIDController.setFF(SwerveModuleConstants.kFModuleDriveController);
    this.drivingPIDController.setIZone(SwerveModuleConstants.kIZoneModuleDriveController);
    this.drivingPIDController.setOutputRange(-1,1);

    this.turningPIDController.setP(SwerveModuleConstants.kPModuleTurningController);
    this.turningPIDController.setI(SwerveModuleConstants.kIModuleTurningController);
    this.turningPIDController.setD(SwerveModuleConstants.kDModuleTurningController);
    this.turningPIDController.setFF(SwerveModuleConstants.kFModuleTurningController);
    this.turningPIDController.setIZone(SwerveModuleConstants.kIZoneModuleTurningController);
    this.turningPIDController.setOutputRange(-.2, .2);
    this.m_driveMotor.burnFlash();
    this.m_turningMotor.burnFlash();

    CANcoderConfiguration turningAbsoluteEncoderConfig = new CANcoderConfiguration();
    MagnetSensorConfigs magnetConfigs = new MagnetSensorConfigs();
    magnetConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    magnetConfigs.MagnetOffset = 0.0f;
    magnetConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    turningAbsoluteEncoderConfig.MagnetSensor = magnetConfigs;
    this.turningAbsoluteEncoder.getConfigurator().apply(turningAbsoluteEncoderConfig);
    Timer.delay(1);

    this.absoluteEncoderOffset = turningEncoderOffset;
    resetTurningMotorToAbsolute(turningEncoderOffset);
  }

  public double getBusVoltage() {
    return this.m_driveMotor.getBusVoltage();
  }

  public void resetTurningMotorToAbsolute(double turningEncoderOffset) {
    this.turningRelativeEncoder.setPosition((this.turningAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() - turningEncoderOffset) * SwerveModuleConstants.kTurningGearRatio);
  
  }

  private double getTurningEncoderAngleRadiens() {
    return Math.toRadians(getTurningEncoderAngleDegrees().getDegrees());
  }

  private double degreesToCPR(double degrees) {
    return (degrees / 360) * SwerveModuleConstants.kNeoEncoderCPR;
  }

  public Rotation2d getTurningEncoderAngleDegrees() {
    return Rotation2d.fromDegrees(this.turningRelativeEncoder.getPosition() * 360 / SwerveModuleConstants.kTurningGearRatio);//this.turningAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360);
  };

  private void setHeadingInDegrees(Rotation2d optimizedDesiredRotation){
    double desiredDegrees = optimizedDesiredRotation.getDegrees(); 
    double desiredEncoderRotation = (desiredDegrees / 360) * SwerveModuleConstants.kTurningGearRatio;
    turningPIDController.setReference(desiredEncoderRotation, ControlType.kPosition);
  }

  public double getDistance() {
    return (m_driveMotor.getEncoder().getPosition() * SwerveModuleConstants.kNeoEncoderCPR) * SwerveModuleConstants.kDriveEncoderDistancePerPulse;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDistance(), new Rotation2d((this.turningAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() - this.absoluteEncoderOffset) * 360));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveMotorSpeedInMetersPerSecond(), getTurningEncoderAngleDegrees());
  }

  public double getDriveMotorSpeedInMetersPerSecond() {
    return this.driveRelativeEncoder.getVelocity() * SwerveModuleConstants.kDriveConversionVelocityFactor;
  }

  public void setDesiredState(SwerveModuleState desiredState) {
      SwerveModuleState state = SwerveModuleAngleOptimizer.optimize(desiredState, getState().angle);

      double desiredRPM = state.speedMetersPerSecond / SwerveModuleConstants.kDriveConversionVelocityFactor; 
      this.drivingPIDController.setReference(desiredRPM, CANSparkMax.ControlType.kVelocity);
      this.setHeadingInDegrees(state.angle);
  }

}
 

 
 