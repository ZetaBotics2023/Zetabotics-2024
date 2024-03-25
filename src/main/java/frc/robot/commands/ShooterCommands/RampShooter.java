package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystem;

public class RampShooter extends Command{
    private ShooterSubsystem m_shooterSubsystem;
    private boolean inited = false;
    public RampShooter(ShooterSubsystem shooterSubsystem) {
        this.m_shooterSubsystem = shooterSubsystem;
    }

    @Override 
    public void initialize() {
        ShooterConstants.kRampShooter = !ShooterConstants.kRampShooter;
        this.inited = true;
        if(ShooterConstants.kRampShooter) {
            this.m_shooterSubsystem.runAtVoltage(ShooterConstants.kShooterRPM, 6);
        } else {
            this.m_shooterSubsystem.runAtVoltage(0, 0);
        }
    } 

    @Override
    public void execute() {
    
    }

    @Override
    public boolean isFinished() {
        return this.inited;
    }
}
