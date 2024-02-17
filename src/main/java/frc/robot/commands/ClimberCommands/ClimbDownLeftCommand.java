package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem.ClimberSubsystem;

public class ClimbDownLeftCommand extends Command {
    
    private ClimberSubsystem climberSubsystem;

    public ClimbDownLeftCommand(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }
    
    @Override
    public void initialize() {
        this.climberSubsystem.setPercentOutput(ClimberConstants.kClimbDownPercentOutput, 0);
    }

    @Override
    public void end(boolean interrupted) {
        this.climberSubsystem.setPercentOutput(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
