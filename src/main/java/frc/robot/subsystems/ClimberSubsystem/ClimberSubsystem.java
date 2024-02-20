package frc.robot.subsystems.ClimberSubsystem;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase{
    private final CANSparkMax m_leftClimber;
    private final CANSparkMax m_rightClimber;
    private final RelativeEncoder m_leftEncoder;
    private final RelativeEncoder m_rightEncoder;

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

    public void setPercentOutput(double percent) {
        setPercentOutput(percent, percent);
    }

    public void setPercentOutput(double leftPercent, double rightPercent) {
        this.m_leftClimber.set(leftPercent);
        this.m_rightClimber.set(rightPercent);
    }

    public double getLeftMotorPositionRotations() {
        return this.m_leftEncoder.getPosition();
    }

    public double getRightMotorPositionRotations() {
        return this.m_rightEncoder.getPosition();
    }
}