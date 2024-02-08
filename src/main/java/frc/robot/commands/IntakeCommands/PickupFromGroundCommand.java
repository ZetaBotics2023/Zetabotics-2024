package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem.IntakeSensorSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.PivotSubsystem;

/**
 * Command to retrieve a note from the ground
 */
public class PickupFromGroundCommand extends Command {

    private IntakeSubsystem intakeSubsystem;
    private PivotSubsystem pivotSubsystem;
    private IntakeSensorSubsystem intakeSensorSubsystem;

    public PickupFromGroundCommand(IntakeSubsystem intakeSusbsystem, PivotSubsystem pivotSubsystem, IntakeSensorSubsystem intakeSensorSubsystem) {
        this.intakeSubsystem = intakeSusbsystem;
        this.pivotSubsystem = pivotSubsystem;
        this.intakeSensorSubsystem = intakeSensorSubsystem;
        addRequirements(this.intakeSubsystem, this.pivotSubsystem, this.intakeSensorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() { 
        this.pivotSubsystem.setTargetPositionDegrees(IntakeConstants.kGroundPickupPivotRotationDegrees);
        this.intakeSubsystem.runAtRPM(IntakeConstants.kGroundPickupIntakeRPM);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    /**
     * When the command ends, set the pivot's target to the position needed
     * to put the note in the shooter and set the intake to stop running.
     */
    @Override
    public void end(boolean interrupted) {
        this.pivotSubsystem.setTargetPositionDegrees(IntakeConstants.kPassIntoShooterPivotRotationDegrees);
        this.intakeSubsystem.runAtRPM(0);
    }

    /**
     * If our sensor detects a note in our intake, stop the command.
     */
    @Override
    public boolean isFinished() {
        return this.intakeSensorSubsystem.isNoteInIntake();
    }
}
