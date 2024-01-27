package frc.robot.subsystems.ShooterSubsystem;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{
    private final CANSparkMax m_leftShooter; 
    private final CANSparkMax m_rightShooter;
    private final RelativeEncoder m_leftEncoder;
    private final RelativeEncoder m_rightEncoder;
    private final SparkPIDController leftShooterPID;
    private final SparkPIDController rightShooterPID; 
    private double targetVelocityRPM = 0;
    private double finishedRunningTimestamp = 0;


    public ShooterSubsystem(boolean leftShooterRev, boolean rightShooterRev)
    {
        this.m_leftShooter = new CANSparkMax(Constants.ShooterConstants.kLeftShooterMotorControllerID, CANSparkMax.MotorType.kBrushless);
        this.m_rightShooter = new CANSparkMax(Constants.ShooterConstants.kRightShooterMotorControllerID, CANSparkMax.MotorType.kBrushless);
        this.m_leftEncoder = this.m_leftShooter.getEncoder();
        this.m_rightEncoder = this.m_rightShooter.getEncoder();
        
        this.leftShooterPID = this.m_leftShooter.getPIDController();
        this.rightShooterPID = this.m_rightShooter.getPIDController();

        this.m_leftShooter.restoreFactoryDefaults();
        this.m_rightShooter.restoreFactoryDefaults();
        this.m_leftShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        this.m_rightShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        this.m_this.leftShooterPID.setP(ShooterConstants.kPLeftShooterController);
        this.leftShooterPID.setI(ShooterConstants.kILeftShooterController);
        this.leftShooterPID.setD(ShooterConstants.kDLeftShooterController);
        this.leftShooterPID.setIZone(ShooterConstants.kIZoneLeftShooterController);
        this.rightShooterPID.setP(ShooterConstants.kPRightShooterController);
        this.rightShooterPID.setI(ShooterConstants.kIRightShooterController);
        this.rightShooterPID.setD(ShooterConstants.kDRightShooterController);
        this.rightShooterPID.setIZone(ShooterConstants.kIZoneRightShooterController);
           
        this.m_leftShooter.burnFlash();
        this.m_rightShooter.burnFlash();leftShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        this.m_rightShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        this.m_leftShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        this.m_rightShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);

        this.m_leftShooter.setIdleMode(IdleMode.kBrake);
        this.m_rightShooter.setIdleMode(IdleMode.kBrake);
        this.m_leftShooter.setInverted(leftShooterRev);
        this.m_rightShooter.setInverted(rightShooterRev);

        this.m_leftShooter.setSmartCurrentLimit(0);
        this.m_rightShooter.setSmartCurrentLimit(0);

        
        

        }

        public void setTargetVelocityRPM(double rpm) {
        this.targetVelocityRPM = rpm;
        this.leftShooterPID.setReference(this.targetVelocityRPM, ControlType.kVelocity);
        this.rightShooterPID.setReference(this.targetVelocityRPM, ControlType.kVelocity);
    }

    // We want a method to check if the pivot motor is at the correct position of degrees
    public boolean isMotorAtTargetVelocity() {
        return Math.abs(this.m_leftEncoder.getVelocity() - this.targetVelocityRPM) <= this.leftShooterPID.getIZone();
    }
    
    //runs both motors at the same speed for x amount of time (i think)
    public void runAtRPMForTime(double rpm, double seconds) {
        this.leftShooterPID.setReference(rpm, ControlType.kVelocity);
        this.rightShooterPID.setReference(rpm, ControlType.kVelocity);
        this.finishedRunningTimestamp = Timer.getFPGATimestamp() + seconds;
    }


    //method that runs left at the passed rpm and right at that rpm*powerRatio
    public void runAtRPMAndRPMRatio(double rpm) {
        this.leftShooterPID.setReference(rpm, ControlType.kVelocity);
        this.rightShooterPID.setReference(rpm*Constants.ShooterConstants.kShooterPowerRatio, ControlType.kVelocity);
        //i am aware that this is probably wrong ^^^^
    }

    //Checks if both motors are at the desired RPM (i did separate methods because idk how to combine them without it being clunky)
    public boolean isLeftMotorAtTargetVelocity() {
        return (this.m_leftEncoder.getVelocity() - this.targetVelocityRPM) <= this.leftShooterPID.getIZone();
    }

    public boolean isRightMotorAtTargetVelocity() {
        return (this.m_rightEncoder.getVelocity() - this.targetVelocityRPM) <= this.rightShooterPID.getIZone();
    }

}



