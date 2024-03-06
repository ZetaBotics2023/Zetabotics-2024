package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
public class ShootIntoAmpWithIntakeCommand extends Command {
    private IntakeSubsystem intakeSubsystem;
    private PivotSubsystem pivotSubsystem;
    private IntakeSensorSubsystem intakeSensorSubsystem;

    public ShootIntoAmpWithIntakeCommand(IntakeSubsystem intakeSusbsystem, PivotSubsystem pivotSubsystem, IntakeSensorSubsystem intakeSensorSubsystem) {
        this.intakeSubsystem = intakeSusbsystem;
        this.pivotSubsystem = pivotSubsystem;
        this.intakeSensorSubsystem = intakeSensorSubsystem;
        addRequirements(this.intakeSubsystem, this.pivotSubsystem, this.intakeSensorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() { 
        this.pivotSubsystem.setTargetPositionDegrees(IntakeConstants.kShootInAmpPivotRotationDegrees);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(this.pivotSubsystem.isMotorAtTargetRotationLarge()) {
            this.intakeSubsystem.runAtRPM(IntakeConstants.kShootInAmpIntakeRPM);
        }
       
    }

    /**
     * When the command ends, stop the intake.
     */
    @Override
    public void end(boolean interrupted) {
        this.intakeSubsystem.runAtRPM(0);
        this.pivotSubsystem.setTargetPositionDegrees(0);
    }

    /**
     * End the command if there's not a note in the intake AND the shooter time has elapsed
     */
    @Override
    public boolean isFinished() {
        return false;
    } 
}
