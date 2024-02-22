package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystem;


public class RampShooterAtDifferentSpeedAutonCommand extends Command {
    private ShooterSubsystem shooterSubsystem;

    public RampShooterAtDifferentSpeedAutonCommand(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        this.shooterSubsystem.runAtRPMAndRPMRatio(ShooterConstants.kShooterRPM);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
