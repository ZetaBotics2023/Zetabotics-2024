package frc.robot.commands.ClimberCommands;

import frc.robot.subsystems.IntakeSubsystem.IntakeSensorSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.PivotSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ClimberSubsystem.ClimberSubsystem;
public class MotorsAtSamePositionCommand extends Command{
      
    private IntakeSubsystem intakeSubsystem;
    private PivotSubsystem pivotSubsystem;
    private IntakeSensorSubsystem intakeSensorSubsystem;
    private ClimberSubsystem climberSubsystem;
    public MotorsAtSamePositionCommand(ClimberSubsystem climberSubsystem, IntakeSubsystem intakeSusbsystem, PivotSubsystem pivotSubsystem, IntakeSensorSubsystem intakeSensorSubsystem) {
        
        this.intakeSubsystem = intakeSusbsystem;
        this.pivotSubsystem = pivotSubsystem;
        this.intakeSensorSubsystem = intakeSensorSubsystem;
        this.climberSubsystem = climberSubsystem;
        addRequirements(this.intakeSubsystem, this.pivotSubsystem, this.intakeSensorSubsystem, this.climberSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() { 
        this.climberSubsystem.setTargetPositionRotations(Constants.ClimberConstants.kPassIntoClimberPositionRotationDegrees);
    }

    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.climberSubsystem.setTargetPositionRotations(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.climberSubsystem.isLeftMotorAtTargetRotation() && this.climberSubsystem.isRighttMotorAtTargetRotation();
    }
}
