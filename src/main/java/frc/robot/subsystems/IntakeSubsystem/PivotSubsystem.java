package frc.robot.subsystems.IntakeSubsystem;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

/**
 * This subsystem allows us to control the pivot angle
 * of the intake. To control the intake itself, use
 * the intake subsystem.
 */
public class PivotSubsystem extends SubsystemBase {
    private TalonFX m_leftPivot;
    private TalonFX m_rightPivot;

    private TalonFXConfiguration m_leftConfig;
    private TalonFXConfiguration m_rightConfig;
    private PositionVoltage positionalControl = new PositionVoltage(IntakeConstants.kPivotThroughBoreZeroOffset).withSlot(0); 

    private CANcoder pivotEncoder;

    private final Slot0Configs slot0Configs = new Slot0Configs();

    private double targetPositionDegrees = 0;

    public PivotSubsystem() {
        this.m_leftPivot = new TalonFX(IntakeConstants.kLeftPivotID);
        this.m_rightPivot = new TalonFX(IntakeConstants.kRightPivotID);
                
        this.pivotEncoder = new CANcoder(IntakeConstants.kPivotEncoderID);
        CANcoderConfiguration pivotEncoderConfig = new CANcoderConfiguration();
        MagnetSensorConfigs magnetConfigs = new MagnetSensorConfigs();
        magnetConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        magnetConfigs.MagnetOffset = 0.0f;
        magnetConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;//Clockwise_Positive;
        pivotEncoderConfig.MagnetSensor = magnetConfigs;
        this.pivotEncoder.getConfigurator().apply(pivotEncoderConfig);
        this.pivotEncoder.setPosition(this.pivotEncoder.getAbsolutePosition().getValueAsDouble());

        this.m_leftConfig = new TalonFXConfiguration();
        this.m_rightConfig = new TalonFXConfiguration();

        this.slot0Configs.kP = 100;//IntakeConstants.kPPivotController;
        this.slot0Configs.kI = IntakeConstants.kIPivotController;
        this.slot0Configs.kD = IntakeConstants.kDPivotController;

        this.m_leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        this.m_rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        this.m_leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        this.m_rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        this.m_leftConfig.Feedback.FeedbackRemoteSensorID = this.pivotEncoder.getDeviceID();
        this.m_rightConfig.Feedback.FeedbackRemoteSensorID = this.pivotEncoder.getDeviceID();
        this.m_leftConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        this.m_rightConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        this.m_leftConfig.Feedback.SensorToMechanismRatio = 1.0;
        this.m_rightConfig.Feedback.SensorToMechanismRatio = 1.0;
        this.m_leftConfig.Feedback.RotorToSensorRatio = IntakeConstants.kPivotGearRatio;
        this.m_rightConfig.Feedback.RotorToSensorRatio = IntakeConstants.kPivotGearRatio;

        this.m_leftPivot.getConfigurator().apply(this.m_leftConfig);
        this.m_rightPivot.getConfigurator().apply(this.m_rightConfig);
        this.m_leftPivot.getConfigurator().apply(this.slot0Configs);
        this.m_rightPivot.getConfigurator().apply(this.slot0Configs);
        this.m_rightPivot.setControl(new StrictFollower(IntakeConstants.kLeftPivotID));

        Timer.delay(1);
    }

    public void periodic() {
        SmartDashboard.putNumber("Pivot Angle(no offset)", rotationsToDegrees(this.m_leftPivot.getPosition().getValueAsDouble()));// rotationsToDegrees(this.m_leftPivot.getPosition().getValueAsDouble()));
        SmartDashboard.putNumber("Pivot Angle(offset)", rotationsToDegrees(this.pivotEncoder.getAbsolutePosition().getValueAsDouble()));// + IntakeConstants.kPivotThroughBoreZeroOffset);
        SmartDashboard.putNumber("Desired Pivot Angle", targetPositionDegrees);


        this.positionalControl.Slot = 0;
       
        //targetPositionDegrees)); //IntakeConstants.kPivotThroughBoreZeroOffset));
        this.m_leftPivot.setControl(this.positionalControl.withPosition(
            degreesToRotations(targetPositionDegrees)));
    }

    public void setTargetPositionDegrees(double desiredDegrees) {
        this.targetPositionDegrees = desiredDegrees;
    }

    /**
     * Converts degrees to rotations
     * @param degrees The amount of degrees
     * @return Rotations
     */
    private double degreesToRotations(double degrees) {
        return (degrees / 360.0);
    }

    /**
     * Converts rotations to degrees
     * @param rotations The amount of rotations
     * @return Degrees
     */
    private double rotationsToDegrees(double rotations) {
        return (rotations * 360.0);
    }

    /**
     * Returns whether or not the PID has succeeded in bringing
     * the pivot to the desired rotation within the PID's IZone.
     * @return If the pivot's rotation is within the IZone of the desired rotation
     */
    
    public boolean isMotorAtTargetRotation() {
        return Math.abs(rotationsToDegrees(this.m_leftPivot.getPosition().getValueAsDouble()) - (this.targetPositionDegrees + IntakeConstants.kPivotThroughBoreZeroOffset)) <= IntakeConstants.kPivotRotationToleranceDegrees;
    }

    public boolean isMotorAtTargetRotationLarge() {
        return Math.abs(rotationsToDegrees(this.m_leftPivot.getPosition().getValueAsDouble()) - this.targetPositionDegrees + (IntakeConstants.kPivotThroughBoreZeroOffset)) <= 20;
    }
}   
 