package frc.robot.subsystems.IntakeSubsystem;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

/**
 * This subsystem allows us control of our intake rollers.
 * To control the pivot angle, use the pivot subsystem.
 */
public class IntakeSubsystem extends SubsystemBase {

    private final CANSparkMax m_intake;
    private final RelativeEncoder m_intakeEncoder;
    private final SparkPIDController intakePID;
    private double targetRPM = 0;
    private double targetPositionRotations;
    

    public IntakeSubsystem(boolean intakeMotorRev){
        this.m_intake = new CANSparkMax(Constants.IntakeConstants.kIntakeMotorControllerID, CANSparkMax.MotorType.kBrushless);
        this.m_intakeEncoder = this.m_intake.getEncoder();
        this.intakePID = this.m_intake.getPIDController();

        this.m_intake.restoreFactoryDefaults();
        this.m_intake.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        this.m_intake.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        this.m_intake.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 20);
    
        this.m_intake.setIdleMode(IdleMode.kBrake);
        this.m_intake.setInverted(intakeMotorRev);
        // This may need to be changed
        this.m_intake.setSmartCurrentLimit(20);
        
        this.intakePID.setP(IntakeConstants.kPIntakeVelocityController, 0);
        this.intakePID.setI(IntakeConstants.kIIntakeVelocityController, 0);
        this.intakePID.setD(IntakeConstants.kDIntakeVelocityController, 0);
        this.intakePID.setIZone(IntakeConstants.kIZoneIntakeVelocityController, 0);

        this.intakePID.setP(IntakeConstants.kPIntakePositionController, 1);
        this.intakePID.setI(IntakeConstants.kIIntakePositionController, 1);
        this.intakePID.setD(IntakeConstants.kDIntakePositionController, 1);
        this.intakePID.setIZone(IntakeConstants.kIZoneIntakePositionController, 1);

        this.m_intake.burnFlash();
    }

    @Override 
    public void periodic() {
        SmartDashboard.putNumber("Desired Intake Speed", targetRPM);
        SmartDashboard.putNumber("Actully Intake speed", this.m_intakeEncoder.getVelocity() / IntakeConstants.kIntakeGearRatio);
        SmartDashboard.putNumber("Desired Intake Pos", targetPositionRotations);
    }

    /**
     * Runs the motors at a specific RPM indefinitely.
     * @param rpm The target RPM of the rollers.
     * @apiNote USE FOR TELEOP
     */
    public void runAtRPM(double rpm) {             
        this.targetRPM = rpm;
        if(rpm == 0) {
            setTargetPoseitionRotations(this.m_intakeEncoder.getPosition() /  IntakeConstants.kIntakeGearRatio);
        } else {
            this.intakePID.setReference(rpm * IntakeConstants.kIntakeGearRatio, ControlType.kVelocity, 0);
        } 
    }

    public void runAtPower(double percent) {
       if(percent == 0) {
            this.m_intake.set(percent);
            setTargetPoseitionRotations(this.m_intakeEncoder.getPosition());
        } else {
            this.m_intake.set(percent);
        }
    }

    /**
     * Runs the motor to a specific position in rotations
     * @param rpm The target RPM of the rollers.
     * @apiNote USE FOR TELEOP
     */
    private void setTargetPoseitionRotations(double rotations) {
        rotations = this.targetPositionRotations;
        this.intakePID.setReference(targetPositionRotations, ControlType.kPosition, 1);
    }

    public double getTargetRPM() {
        return this.targetRPM;
    }
}
