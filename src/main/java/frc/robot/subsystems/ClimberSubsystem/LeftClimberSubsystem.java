package frc.robot.subsystems.ClimberSubsystem;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

/*
 * This subsystem allows us to control our climber motors
 */
public class LeftClimberSubsystem extends SubsystemBase{
    private final CANSparkMax m_leftClimber;
    private final RelativeEncoder m_leftEncoder;
    private double desiredLeftMotorPercent = 0;


    public LeftClimberSubsystem(boolean leftClimberRev) {
        this.m_leftClimber = new CANSparkMax(Constants.ClimberConstants.kLeftClimberMotorControllerID, CANSparkMax.MotorType.kBrushless);
        this.m_leftEncoder = this.m_leftClimber.getEncoder();

        this.m_leftClimber.restoreFactoryDefaults();
        this.m_leftClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        this.m_leftClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50000);
        this.m_leftClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 50000);
        //this.m_leftClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 50000);
        //this.m_rightClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 50000);
        //this.m_leftClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 50000);
        //this.m_rightClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 50000);
        //this.m_leftClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 50000);
        //this.m_rightClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 50000);

        this.m_leftClimber.setIdleMode(IdleMode.kBrake);
        this.m_leftClimber.setInverted(leftClimberRev);

        this.m_leftClimber.setSmartCurrentLimit(40);

        this.m_leftClimber.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.kClimberMaxHeight);
        this.m_leftClimber.setSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.kClimberMinHeight);
        
    
        this.m_leftClimber.enableSoftLimit(SoftLimitDirection.kForward, true);
        this.m_leftClimber.enableSoftLimit(SoftLimitDirection.kReverse, true);

        this.m_leftClimber.burnFlash();
    }



    /*
     * Runs both motors at a given percentage
     */
    public void setPercentOutput(double percent) {
        this.m_leftClimber.set(percent);
    }


    /*
     * Returns how many rotations the left motor has completed
     */
    public double getLeftMotorPositionRotations() {
        return this.m_leftEncoder.getPosition();
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

}
