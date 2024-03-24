package frc.robot.subsystems.IntakeSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

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
public class REVPivotSubsystem extends SubsystemBase {
    
    private final CANSparkMax m_pivot;
    private final SparkAbsoluteEncoder m_pivotAbsEncoder;
    private final RelativeEncoder m_pivotRelativeEncoder;
    private final SparkPIDController pivotPID;
    private double targetPositionDegrees = 0.0;

    public REVPivotSubsystem(boolean pivotMotorRev) {
        this.m_pivot = new CANSparkMax(Constants.IntakeConstants.kPivotMotorControllerID, CANSparkMax.MotorType.kBrushless);
        this.m_pivotRelativeEncoder = this.m_pivot.getEncoder();
        this.m_pivotAbsEncoder = this.m_pivot.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        this.pivotPID = this.m_pivot.getPIDController();
     
        this.m_pivot.restoreFactoryDefaults();      
        this.m_pivot.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 50000);
        this.m_pivot.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        this.m_pivot.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 50000);
        
        this.m_pivot.setIdleMode(IdleMode.kBrake);
        this.m_pivot.setInverted(pivotMotorRev);

        // This may need to be changed
        // Temp voltage for safty, increase onces tuned.
        this.m_pivot.setSmartCurrentLimit(40);

        this.pivotPID.setFeedbackDevice(this.m_pivotAbsEncoder);
        this.pivotPID.setP(IntakeConstants.kPPivotController);
        this.pivotPID.setI(IntakeConstants.kIPivotController);
        this.pivotPID.setD(IntakeConstants.kDPivotController);
        this.pivotPID.setFF(IntakeConstants.kFPivotController);
        this.pivotPID.setIZone(IntakeConstants.kIZoneIntakeVelocityController);
        this.pivotPID.setOutputRange(-1, 1);
        this.m_pivot.burnFlash();
        Timer.delay(1);
    }

    /**
     * Periodic to output helpful values to SmartDashboard
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Desired pivot angle", targetPositionDegrees);
        SmartDashboard.putNumber("Actully ABS pivot position degrees", rotationsToDegrees(this.m_pivotAbsEncoder.getPosition()) - IntakeConstants.kPivotThroughBoreZeroOffset);
        //SmartDashboard.putNumber("Actully rel pivot pos degrees", rotationsToDegrees(this.m_pivotRelativeEncoder.getPosition()) / IntakeConstants.kPivotGearRatio);
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
     * Sets our PID to target the given rotation in degrees
     * @param desiredDegrees The desired position of the pivot
     */
    public void setTargetPositionDegrees(double desiredDegrees) {
        pivotPID.setReference(degreesToRotations(desiredDegrees + IntakeConstants.kPivotThroughBoreZeroOffset), CANSparkMax.ControlType.kPosition);
        this.targetPositionDegrees = desiredDegrees;
    }
    
    /**
     * Returns whether or not the PID has succeeded in bringing
     * the pivot to the desired rotation within the PID's IZone.
     * @return If the pivot's rotation is within the IZone of the desired rotation
     */
    public boolean isMotorAtTargetRotation() {
        return Math.abs(rotationsToDegrees(this.m_pivotAbsEncoder.getPosition()) - this.targetPositionDegrees) <= IntakeConstants.kPivotRotationToleranceDegrees;
    }

    public boolean isMotorAtTargetRotationLarge() {
        return Math.abs(rotationsToDegrees(this.m_pivotAbsEncoder.getPosition()) - this.targetPositionDegrees) <= 20;
    }

    public boolean isPivotAboveAutonPickupThreshold() {
        return rotationsToDegrees(this.m_pivotAbsEncoder.getPosition()) >= IntakeConstants.kGroundPickupMinimumPosition;
    }    
}
 