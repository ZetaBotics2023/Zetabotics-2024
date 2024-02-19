package frc.robot.subsystems.ShooterSubsystem;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
        
        this.leftShooterPID.setP(ShooterConstants.kPShooterController);
        this.leftShooterPID.setI(ShooterConstants.kIShooterController);
        this.leftShooterPID.setD(ShooterConstants.kDShooterController);
        this.leftShooterPID.setIZone(ShooterConstants.kIZoneShooterController);

        this.rightShooterPID.setP(ShooterConstants.kPShooterController);
        this.rightShooterPID.setI(ShooterConstants.kIShooterController);
        this.rightShooterPID.setD(ShooterConstants.kDShooterController);
        this.rightShooterPID.setIZone(ShooterConstants.kIZoneShooterController);

        this.m_leftShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        this.m_rightShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        this.m_leftShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        this.m_rightShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        this.m_leftShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        this.m_rightShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);

        this.m_leftShooter.setIdleMode(IdleMode.kCoast);
        this.m_rightShooter.setIdleMode(IdleMode.kCoast);
        this.m_leftShooter.setInverted(leftShooterRev);
        this.m_rightShooter.setInverted(rightShooterRev);

        this.m_leftShooter.setSmartCurrentLimit(40);
        this.m_rightShooter.setSmartCurrentLimit(40);

        //TODO: Check if this helps consistency
        this.m_leftShooter.enableVoltageCompensation(12);
        this.m_rightShooter.enableVoltageCompensation(12);
           
        this.m_leftShooter.burnFlash();
        this.m_rightShooter.burnFlash();
        }  

        public void setTargetVelocityRPM(double rpm) {
            SmartDashboard.putNumber("Desired Shooter RPM", rpm);
            this.targetVelocityRPM = rpm;
            if(rpm == 0) {
                this.m_leftShooter.set(0);
                this.m_rightShooter.set(0);
            } else {
                this.leftShooterPID.setReference(this.targetVelocityRPM, ControlType.kVelocity);
                this.rightShooterPID.setReference(this.targetVelocityRPM, ControlType.kVelocity);
            }
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Actully Shooter RPM Right", this.m_rightEncoder.getVelocity());
        SmartDashboard.putNumber("Actully Shooter RPM Left", this.m_leftEncoder.getVelocity());
    }

    //runs both motors at the same speed for x amount of time (i think)
    public void runAtRPMForTime(double rpm, double seconds) {
        this.leftShooterPID.setReference(rpm, ControlType.kVelocity);
        this.rightShooterPID.setReference(rpm, ControlType.kVelocity);
        this.finishedRunningTimestamp = Timer.getFPGATimestamp() + seconds;
    }


    //method that runs left at the passed rpm and right at that rpm*powerRatio
    public void runAtRPMAndRPMRatio(double rpm) {
        this.targetVelocityRPM = rpm;
        SmartDashboard.putNumber("Desired Shooter RPM", rpm);
        if(rpm == 0) {
            this.m_leftShooter.set(0);
            this.m_rightShooter.set(0);
        } else {
            this.leftShooterPID.setReference(rpm, ControlType.kVelocity);
            this.rightShooterPID.setReference(rpm*Constants.ShooterConstants.kShooterPowerRatio, ControlType.kVelocity);
        }
       
    }

    //Checks if both motors are at the desired RPM (i did separate methods because idk how to combine them without it being clunky)
    public boolean isLeftMotorAtTargetVelocity() {
        return Math.abs(this.m_leftEncoder.getVelocity() - this.targetVelocityRPM) <= ShooterConstants.kShooterRPMTolorence;
    }

    public boolean isRightMotorAtTargetVelocity() {
        return Math.abs(this.m_rightEncoder.getVelocity() - this.targetVelocityRPM) <= ShooterConstants.kShooterRPMTolorence;
    }

    public boolean isRightMotorAtTargetRatioVelocity() {
        return Math.abs(this.m_rightEncoder.getVelocity() - this.targetVelocityRPM * ShooterConstants.kShooterPowerRatio) <= ShooterConstants.kShooterRPMTolorence;
    }

}



