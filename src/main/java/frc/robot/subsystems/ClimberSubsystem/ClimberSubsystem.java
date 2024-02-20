package frc.robot.subsystems.ClimberSubsystem;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase{
    private final CANSparkMax m_leftClimber;
    private final CANSparkMax m_rightClimber;
    private final RelativeEncoder m_leftEncoder;
    private final RelativeEncoder m_rightEncoder;
    private double desiredLeftMotorPercent = 0;
    private double desiredRightMotorPercent = 0;


    public ClimberSubsystem(boolean leftClimberRev, boolean rightClimberRev) {
        this.m_leftClimber = new CANSparkMax(Constants.ClimberConstants.kLeftClimberMotorControllerID, CANSparkMax.MotorType.kBrushless);
        this.m_rightClimber = new CANSparkMax(Constants.ClimberConstants.kRightClimberMotorControllerID, CANSparkMax.MotorType.kBrushless);
        this.m_leftEncoder = this.m_leftClimber.getEncoder();
        this.m_rightEncoder = this.m_rightClimber.getEncoder();

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

        this.m_leftClimber.setSmartCurrentLimit(40);
        this.m_rightClimber.setSmartCurrentLimit(40);

           
        this.m_leftClimber.burnFlash();
        this.m_rightClimber.burnFlash();
    }

    @Override
    public void periodic() {
        if(!shouldLeftMotorMove(this.desiredLeftMotorPercent)) {
            setPercentOutput(0, this.desiredRightMotorPercent);
        }

        if(!shouldRightMotorMove(this.desiredRightMotorPercent)) {
            setPercentOutput(this.desiredLeftMotorPercent, 0);
        }
    }

    public void setPercentOutput(double percent) {
        setPercentOutput(percent, percent);
    }

    public void setPercentOutput(double leftPercent, double rightPercent) {
        if(shouldLeftMotorMove(leftPercent)) {
            this.desiredLeftMotorPercent = leftPercent;
        } else {
            this.desiredLeftMotorPercent = 0;
        }

        if(shouldRightMotorMove(rightPercent)) {
            this.desiredRightMotorPercent = rightPercent;
        } else {
            this.desiredRightMotorPercent = 0;
        }
        
        this.m_leftClimber.set(this.desiredLeftMotorPercent);
        this.m_rightClimber.set(this.desiredRightMotorPercent);
    }

    public double getLeftMotorPositionRotations() {
        return this.m_leftEncoder.getPosition();
    }

    public double getRightMotorPositionRotations() {
        return this.m_rightEncoder.getPosition();
    }

    /***
     * 
     * @param direction + for up - for down
     * @return Wether the left motor should be able to move
     */
    private boolean shouldLeftMotorMove(double direction) {
        if(direction < 0) {
            return this.m_leftEncoder.getPosition() > ClimberConstants.kClimberMinHeight;
        } else if(direction > 0) {
            return this.m_leftEncoder.getPosition() < ClimberConstants.kClimberMaxHeight;
        }
        return false;
    }

    /***
     * 
     * @param direction 1 for up -1 for down
     * @return Wether the right motor should be able to move
     */
    private boolean shouldRightMotorMove(double direction) {
        if(direction < 0) {
            return this.m_rightEncoder.getPosition() > ClimberConstants.kClimberMinHeight;
        } else if(direction > 0) {
            return this.m_rightEncoder.getPosition() < ClimberConstants.kClimberMaxHeight;
        }
        return false;
    }
}
