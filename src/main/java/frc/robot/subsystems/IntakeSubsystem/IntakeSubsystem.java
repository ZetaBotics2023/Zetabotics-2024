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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final CANSparkMax m_intake;
    private final RelativeEncoder m_intakeEncoder;
    private final SparkPIDController intakePID;
    private double finishedRunningTimestamp = 0;

    public IntakeSubsystem(boolean intakeMotorRev) {
        this.m_intake = new CANSparkMax(Constants.IntakeConstants.kIntakeMotorControllerID, CANSparkMax.MotorType.kBrushless);
        this.m_intakeEncoder = this.m_intake.getEncoder();
        this.intakePID = this.m_intake.getPIDController();

        this.m_intake.restoreFactoryDefaults();
        this.m_intake.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        this.m_intake.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        this.m_intake.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
    
        this.m_intake.setIdleMode(IdleMode.kBrake);
        this.m_intake.setInverted(intakeMotorRev);
        // This may need to be changed
        // Temp voltage for safty, increase onces tuned.
        this.m_intake.setSmartCurrentLimit(20);
        
        this.intakePID.setP(IntakeConstants.kPIntakeController);
        this.intakePID.setI(IntakeConstants.kIIntakeController);
        this.intakePID.setD(IntakeConstants.kDIntakeController);
        this.intakePID.setIZone(IntakeConstants.kIZoneIntakeController);

        this.m_intake.burnFlash();
    }
    public void runAtSpeedForTime(double rpm, double seconds) {
        this.intakePID.setReference(rpm, ControlType.kVelocity);
        this.finishedRunningTimestamp = Timer.getFPGATimestamp() + seconds;
    }

    public boolean isRPMTimeReached() {
        boolean isTimeReached = Timer.getFPGATimestamp() - this.finishedRunningTimestamp >= 0;
        if (isTimeReached) this.intakePID.setReference(0, ControlType.kVelocity);
        return isTimeReached;
    }
}
