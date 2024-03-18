package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem.IntakeSensorSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.PivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem.REVPivotSubsystem;

/**
 * Command to retrieve a note from the ground
 */
public class PivotIntakeDown extends Command {

    private IntakeSubsystem intakeSubsystem;
    private PivotSubsystem pivotSubsystem;
    private IntakeSensorSubsystem intakeSensorSubsystem;

    public PivotIntakeDown(IntakeSubsystem intakeSusbsystem, PivotSubsystem pivotSubsystem, IntakeSensorSubsystem intakeSensorSubsystem) {
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
        
    }

    /**
     * If our sensor detects a note in our intake, stop the command.
     */
    @Override
    public boolean isFinished() {
        return this.pivotSubsystem.isMotorAtTargetRotation();
    }
}
