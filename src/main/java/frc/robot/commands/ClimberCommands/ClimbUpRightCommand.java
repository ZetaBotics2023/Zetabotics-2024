package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem.RightClimberSubsystem;

public class ClimbUpRightCommand extends Command {
    
    private RightClimberSubsystem climberSubsystem;

    public ClimbUpRightCommand(RightClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        //addRequirements(climberSubsystem);
    }
    
    @Override
    public void initialize() {
        this.climberSubsystem.setPercentOutput(ClimberConstants.kClimbUpPercentOutput);
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
