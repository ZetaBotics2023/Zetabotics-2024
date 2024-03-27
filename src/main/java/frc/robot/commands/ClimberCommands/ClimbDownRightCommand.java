package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem.RightClimberSubsystem;

public class ClimbDownRightCommand extends Command {
    
    private RightClimberSubsystem climberSubsystem;

    public ClimbDownRightCommand(RightClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
       // addRequirements(climberSubsystem);
    }
    
    @Override
    public void initialize() {
        this.climberSubsystem.setPercentOutput(ClimberConstants.kClimbDownPercentOutput);
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
