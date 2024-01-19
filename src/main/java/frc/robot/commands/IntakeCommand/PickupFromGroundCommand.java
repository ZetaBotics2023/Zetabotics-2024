package frc.robot.commands.IntakeCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.PivotSubsystem;

public class PickupFromGroundCommand extends Command {
    private IntakeSubsystem intakeSubsystem;
    private PivotSubsystem pivotSubsystem;
    public PickupFromGroundCommand(IntakeSubsystem intakeSusbsystem, PivotSubsystem pivotSubsystem) {
        
        this.intakeSubsystem = intakeSusbsystem;
        this.pivotSubsystem = pivotSubsystem;
        addRequirements(this.intakeSubsystem, this.pivotSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() { 
        //this.intakeSubsystem.runAtSpeedForTime(IntakeConstants.kGroundPickupIntakeRPM, ); 
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
