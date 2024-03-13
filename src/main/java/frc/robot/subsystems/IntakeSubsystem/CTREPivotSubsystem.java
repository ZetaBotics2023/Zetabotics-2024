package frc.robot.subsystems.IntakeSubsystem;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
public class CTREPivotSubsystem extends SubsystemBase {
    private TalonFX m_leftPivot;
    private TalonFX m_rightPivot;

    private TalonFXConfiguration m_leftConfig;
    private TalonFXConfiguration m_rightConfig;

    private final Slot0Configs slot0Configs = new Slot0Configs();

    private CANSparkMax pivotThroughBoreSpark;
    private SparkAbsoluteEncoder pivotThroughBore;

    private ProfiledPIDController pivotPID;

    private double targetPositionDegrees = 0;

    public CTREPivotSubsystem() {
        this.m_leftPivot = new TalonFX(25);
        this.m_rightPivot = new TalonFX(26);

        this.m_leftConfig = new TalonFXConfiguration();
        this.m_rightConfig = new TalonFXConfiguration();

        this.slot0Configs.kP = IntakeConstants.kPPivotController;
        this.slot0Configs.kI = IntakeConstants.kIPivotController;
        this.slot0Configs.kD = IntakeConstants.kDPivotController;

        this.m_leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        this.m_rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        this.m_leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        this.m_rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        this.m_rightPivot.setControl(new StrictFollower(25));

        this.m_leftPivot.getConfigurator().apply(this.m_leftConfig);
        this.m_rightPivot.getConfigurator().apply(this.m_rightConfig);
        this.m_leftPivot.getConfigurator().apply(this.slot0Configs);
        this.m_rightPivot.getConfigurator().apply(this.slot0Configs);

        this.pivotThroughBoreSpark = new CANSparkMax(IntakeConstants.kPivotMotorControllerID, MotorType.kBrushless);
        this.pivotThroughBore = pivotThroughBoreSpark.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        
        this.pivotPID = new ProfiledPIDController(IntakeConstants.kPPivotController, IntakeConstants.kIPivotController, 
            IntakeConstants.kDPivotController, IntakeConstants.kPivotContraints);
        this.pivotPID.setTolerance(IntakeConstants.kPivotAngleTolorence);
        this.pivotPID.setIntegratorRange(1, 1);
        this.pivotPID.reset(rotationsToDegrees(this.pivotThroughBore.getPosition()));

        Timer.delay(1);
    }

    public void periodic() {
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
}   
 