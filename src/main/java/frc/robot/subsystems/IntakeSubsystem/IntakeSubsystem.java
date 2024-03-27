package frc.robot.subsystems.IntakeSubsystem;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.Timer;
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

    private int numberOfPowerSets = 0;
    private int numberOfVelocitySets = 0;
    private int numberOfRotationSets = 0;
    private boolean hasStopedIntake;

    private double endTime = 0;
    private double currentTime = -1;


    

    public IntakeSubsystem(boolean intakeMotorRev){
        this.m_intake = new CANSparkMax(Constants.IntakeConstants.kIntakeMotorControllerID, CANSparkMax.MotorType.kBrushless);
        this.m_intakeEncoder = this.m_intake.getEncoder();
        this.intakePID = this.m_intake.getPIDController();

        this.m_intake.restoreFactoryDefaults();
        this.m_intake.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        this.m_intake.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50000);
        this.m_intake.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 50000);
        //this.m_intake.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 50000);
        //this.m_intake.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 50000);
        //this.m_intake.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 50000);

    
        this.m_intake.setIdleMode(IdleMode.kBrake);
        this.m_intake.setInverted(intakeMotorRev);
        // This may need to be changed
        this.m_intake.setSmartCurrentLimit(40);
        
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
        //SmartDashBoard.putBoolean("Has stopped intake", this.hasStopedIntake);
        //SmartDashBoard.putNumber("this.endTime", this.endTime);
        this.currentTime = Timer.getFPGATimestamp();
        if(this.currentTime > this.endTime && this.endTime != 0 && targetRPM == 0) {
            this.hasStopedIntake = true;
            this.endTime = 0;
            this.currentTime = -1;
            this.m_intake.set(0);
        }

        //SmartDashBoard.putNumber("Desired Intake Speed", targetRPM);
        //SmartDashBoard.putNumber("Actully Intake speed", this.m_intakeEncoder.getVelocity() / IntakeConstants.kIntakeGearRatio);
        //SmartDashBoard.putNumber("Desired Intake Pos", targetPositionRotations);

        //SmartDashBoard.putNumber("Number of times intake power has bean set", numberOfPowerSets);
        //SmartDashBoard.putNumber("Number of times intake rotation has bean set", numberOfRotationSets);
        //SmartDashBoard.putNumber("Number of times intake velocity has bean set", numberOfVelocitySets);
    }

    /**
     * Runs the motors at a specific RPM indefinitely.
     * @param rpm The target RPM of the rollers.
     * @apiNote USE FOR TELEOP
     */
    public void runAtRPM(double rpm) { 
        this.hasStopedIntake = false;  
        this.numberOfVelocitySets++;        
        this.targetRPM = rpm;
        //SmartDashBoard.putBoolean("RMP", rpm == 0);
        if(rpm == 0) {
            this.m_intake.stopMotor();
            this.m_intake.set(.2);
            this.endTime = Timer.getFPGATimestamp() + .5;
        } else {
            this.intakePID.setReference(rpm * IntakeConstants.kIntakeGearRatio, ControlType.kVelocity, 0);
        } 
    }

    public void runAtRPMNorm(double rpm) {   
        if(rpm == 0) {
            this.m_intake.stopMotor();
            this.m_intake.set(0);
        } else {
            this.intakePID.setReference(rpm * IntakeConstants.kIntakeGearRatio, ControlType.kVelocity, 0);
        }
    }



    public void runAtPower(double percent) {
        this.numberOfPowerSets++;
       if(percent == 0) {
            this.m_intake.set(percent);
            setTargetPoseitionRotations(this.m_intakeEncoder.getPosition());
        } else {   
            this.m_intake.set(percent);
        }
        //SmartDashBoard.putNumber("Percent intake", percent);
    }

    /**
     * Runs the motor to a specific position in rotations
     * @param rpm The target RPM of the rollers.
     * @apiNote USE FOR TELEOP
     */
    private void setTargetPoseitionRotations(double rotations) {
        this.numberOfRotationSets++;
        rotations = this.targetPositionRotations;
        this.intakePID.setReference(targetPositionRotations, ControlType.kPosition, 1);
    }

    public double getTargetRPM() {
        return this.targetRPM;
    }
}
