package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem.IntakeSensorSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.PivotSubsystem;

public class ShootIntoAmpWithIntakeCommand extends Command {
    private IntakeSubsystem intakeSubsystem;
    private PivotSubsystem pivotSubsystem;
    private IntakeSensorSubsystem intakeSensorSubsystem;
    private WaitCommand waitCommand;
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
        if(this.pivotSubsystem.isMotorAtTargetRotation() && intakeSubsystem.getTargetRPM() == 0) {
            this.intakeSubsystem.runAtRPM(IntakeConstants.kShootInAmpIntakeRPM);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.pivotSubsystem.setTargetPositionDegrees(IntakeConstants.kPassIntoShooterPivotRotationDegrees);
        this.intakeSubsystem.runAtRPM(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if(!this.intakeSensorSubsystem.isNoteInIntake() && this.waitCommand == null) {
            this.waitCommand = new WaitCommand(IntakeConstants.kShootInAmpIntakeTime);
            this.waitCommand.schedule();
        } 
        if(this.waitCommand != null) {
            if(this.waitCommand.isFinished()) {
                this.waitCommand = null;
                return true;
            }
        }
        return false;
    }
}
