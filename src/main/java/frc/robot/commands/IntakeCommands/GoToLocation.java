package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem.PivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem.REVPivotSubsystem;

public class GoToLocation extends Command {
    private PivotSubsystem pivotSubsystem;

    public GoToLocation(PivotSubsystem pivotSubsystem) {
        this.pivotSubsystem = pivotSubsystem;
        addRequirements(this.pivotSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() { 
        this.pivotSubsystem.setTargetPositionDegrees(170);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.pivotSubsystem.setTargetPositionDegrees(10);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
