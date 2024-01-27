package frc.robot.subsystems.IntakeSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class PivotSubsystem extends SubsystemBase {
    
    private final CANSparkMax m_pivot;
    private final SparkAbsoluteEncoder m_pivotEncoder;
    private final SparkPIDController pivotPID;

    public PivotSubsystem(boolean pivotMotorRev) {
        this.m_pivot = new CANSparkMax(Constants.IntakeConstants.kPivotMotorControllerID, CANSparkMax.MotorType.kBrushless);
        this.m_pivotEncoder = this.m_pivot.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        this.pivotPID = this.m_pivot.getPIDController();

        this.m_pivot.restoreFactoryDefaults();
        this.m_pivot.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        this.m_pivot.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        this.m_pivot.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
    
        this.m_pivot.setIdleMode(IdleMode.kBrake);
        this.m_pivot.setInverted(pivotMotorRev);
        // This may need to be changed
        // Temp voltage for safty, increase onces tuned.
        this.m_pivot.setSmartCurrentLimit(5);
        
        this.pivotPID.setP(IntakeConstants.kPPivotController);
        this.pivotPID.setI(IntakeConstants.kIPivotController);
        this.pivotPID.setD(IntakeConstants.kDPivotController);
        this.pivotPID.setIZone(IntakeConstants.kIZoneIntakeController);

        this.m_pivot.burnFlash();
    }

    private double degreesToRotations(double degrees) {
        return (degrees / 360.0) * IntakeConstants.kPivotGearRatio;
    }
    private double rotationsToDegrees(double rotations) {
        return (rotations * 360.0) / IntakeConstants.kPivotGearRatio;
    }

    // We want a set pviot position in degrees
    public void setTargetPositionDegrees(double degrees) {
        this.targetPositionDegrees = degrees;
        this.pivotPID.setReference(degreesToRotations(this.targetPositionDegrees), ControlType.kPosition);
    }
    // We want a method to check if the pivot motor is at the correct position of degrees
    public boolean isMotorAtTargetRotation() {
        return Math.abs(rotationsToDegrees(this.m_pivotEncoder.getPosition()) - this.targetPositionDegrees) <= this.pivotPID.getIZone();
    }

    
}
 