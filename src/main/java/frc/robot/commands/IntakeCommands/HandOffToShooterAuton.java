package frc.robot.commands.IntakeCommands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem.IntakeSensorSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.PivotSubsystem;

/**
 * Sends a loaded note into our shooter
 */
public class HandOffToShooterAuton extends Command {
    private IntakeSubsystem intakeSubsystem;
    private PivotSubsystem pivotSubsystem;
    private IntakeSensorSubsystem intakeSensorSubsystem;
    private WaitCommand shootWaitTime = null;

    public HandOffToShooterAuton(IntakeSubsystem intakeSusbsystem, PivotSubsystem pivotSubsystem, IntakeSensorSubsystem intakeSensorSubsystem) {
        this.intakeSubsystem = intakeSusbsystem;
        this.pivotSubsystem = pivotSubsystem;
        this.intakeSensorSubsystem = intakeSensorSubsystem;
        addRequirements(this.intakeSubsystem, this.pivotSubsystem, this.intakeSensorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() { 
        this.shootWaitTime = null;
        this.pivotSubsystem.setTargetPositionDegrees(IntakeConstants.kPassIntoShooterPivotRotationDegrees);
        this.intakeSubsystem.runAtRPM(IntakeConstants.kPassIntoShooterIntakeRPM);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
       
    }

    /**
     * When the command ends, stop the intake.
     */
    @Override
    public void end(boolean interrupted) {
        this.intakeSubsystem.runAtRPM(0);
    }

    /**
     * End the command if there's not a note in the intake AND the shooter time has elapsed
     */
    @Override
    public boolean isFinished() {
        if(this.shootWaitTime == null && !this.intakeSensorSubsystem.isNoteInIntake()) {
            this.shootWaitTime = new WaitCommand(ShooterConstants.kShootTimeAuto);
            //SmartDashBoard.putString("HandOffState", "Started Timer");
            this.shootWaitTime.schedule();
        } 
        if(this.shootWaitTime != null) {
            //SmartDashBoard.putString("HandOffState", this.shootWaitTime.isFinished() ? "Ended Timer" : "Timer Still Going");
            return this.shootWaitTime.isFinished();
        }
        return false;
    }
}
