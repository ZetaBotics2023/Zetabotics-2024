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
public class RightClimberSubsystem extends SubsystemBase{
    private final CANSparkMax m_rightClimber;
    private final RelativeEncoder m_rightEncoder;


    public RightClimberSubsystem(boolean rightClimberRev) {
        this.m_rightClimber = new CANSparkMax(Constants.ClimberConstants.kRightClimberMotorControllerID, CANSparkMax.MotorType.kBrushless);
        this.m_rightEncoder = this.m_rightClimber.getEncoder();

        this.m_rightClimber.restoreFactoryDefaults();
        this.m_rightClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        this.m_rightClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50000);
        this.m_rightClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 50000);
        //this.m_rightClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 50000);
        //this.m_rightClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 50000);
        //this.m_rightClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 50000);
        //this.m_rightClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 50000);
        //this.m_rightClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 50000);
        //this.m_rightClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 50000);

        this.m_rightClimber.setIdleMode(IdleMode.kBrake);
        this.m_rightClimber.setInverted(rightClimberRev);

        this.m_rightClimber.setSmartCurrentLimit(40);

        this.m_rightClimber.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.kClimberMaxHeight);
        this.m_rightClimber.setSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.kClimberMinHeight);
        
    
        this.m_rightClimber.enableSoftLimit(SoftLimitDirection.kForward, true);
        this.m_rightClimber.enableSoftLimit(SoftLimitDirection.kReverse, true);

        this.m_rightClimber.burnFlash();
    }



    /*
     * Runs both motors at a given percentage
     */
    public void setPercentOutput(double percent) {
        this.m_rightClimber.set(percent);
    }


    /*
     * Returns how many rotations the right motor has completed
     */
    public double getrightMotorPositionRotations() {
        return this.m_rightEncoder.getPosition();
    }
    /***
     * 
     * @param direction + for up - for down
     * @return Wether the right motor should be able to move
     */
    private boolean shouldrightMotorMove(double direction) {
        if(direction < 0) {
            return this.m_rightEncoder.getPosition() > ClimberConstants.kClimberMinHeight;
        } else if(direction > 0) {
            return this.m_rightEncoder.getPosition() < ClimberConstants.kClimberMaxHeight;
        }
        return false;
    }

}
