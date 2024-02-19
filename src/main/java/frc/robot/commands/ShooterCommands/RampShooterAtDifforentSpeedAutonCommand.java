package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystem;


public class RampShooterAtDifforentSpeedAutonCommand extends Command {
    private ShooterSubsystem shooterSubsystem;

    public RampShooterAtDifforentSpeedAutonCommand(ShooterSubsystem shooterSubsystem){
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
