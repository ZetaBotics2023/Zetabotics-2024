package frc.robot.subsystems.ClimberSubsystem;

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
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class ClimberSubsystem extends SubsystemBase{
    private final CANSparkMax m_leftClimber;
    private final CANSparkMax m_rightClimber;
    private final RelativeEncoder m_leftEncoder;
    private final RelativeEncoder m_rightEncoder;
    private final SparkPIDController leftClimberPID;
    private final SparkPIDController rightClimberPID; 
    private double targetPositionRotations = 0;
    private double finishedRunningTimestamp = 0;

    public ClimberSubsystem(boolean leftClimberRev, boolean rightClimberRev) {
        this.m_leftClimber = new CANSparkMax(Constants.ClimberConstants.kLeftClimberMotorControllerID, CANSparkMax.MotorType.kBrushless);
        this.m_rightClimber = new CANSparkMax(Constants.ClimberConstants.kRightClimberMotorControllerID, CANSparkMax.MotorType.kBrushless);
        this.m_leftEncoder = this.m_leftClimber.getEncoder();
        this.m_rightEncoder = this.m_rightClimber.getEncoder();
        
        this.leftClimberPID = this.m_leftClimber.getPIDController();
        this.rightClimberPID = this.m_rightClimber.getPIDController();

        this.m_leftClimber.restoreFactoryDefaults();
        this.m_rightClimber.restoreFactoryDefaults();
        this.m_leftClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        this.m_rightClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        this.m_leftClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        this.m_rightClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        this.m_leftClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        this.m_rightClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);

        this.m_leftClimber.setIdleMode(IdleMode.kBrake);
        this.m_rightClimber.setIdleMode(IdleMode.kBrake);
        this.m_leftClimber.setInverted(leftClimberRev);
        this.m_rightClimber.setInverted(rightClimberRev);

        this.m_leftClimber.setSmartCurrentLimit(0);
        this.m_rightClimber.setSmartCurrentLimit(0);

        this.leftClimberPID.setP(ClimberConstants.kPLeftClimberController);
        this.leftClimberPID.setI(ClimberConstants.kILeftClimberController);
        this.leftClimberPID.setD(ClimberConstants.kDLeftClimberController);
        //this.leftClimberPID.setF(ClimberConstants.kFLeftClimberController);
        this.leftClimberPID.setIZone(ClimberConstants.kIZoneLeftClimberController);
        this.rightClimberPID.setP(ClimberConstants.kPRightClimberController);
        this.rightClimberPID.setI(ClimberConstants.kIRightClimberController);
        this.rightClimberPID.setD(ClimberConstants.kDRightClimberController);
        this.rightClimberPID.setIZone(ClimberConstants.kIZoneRightClimberController);
           
        this.m_leftClimber.burnFlash();
        this.m_rightClimber.burnFlash();
    }

    public void setTargetPositionRotations(double rotations) {
        this.targetPositionRotations = rotations;
        this.leftClimberPID.setReference((this.targetPositionRotations), ControlType.kPosition);
        this.rightClimberPID.setReference((this.targetPositionRotations), ControlType.kPosition);
    }

    public boolean isLeftMotorAtTargetRotation() {
        return Math.abs((this.m_leftEncoder.getPosition()) - this.targetPositionRotations) <= this.leftClimberPID.getIZone();
    }

    public boolean isRighttMotorAtTargetRotation() {
        return Math.abs((this.m_rightEncoder.getPosition()) - this.targetPositionRotations) <= this.leftClimberPID.getIZone();
    }
}