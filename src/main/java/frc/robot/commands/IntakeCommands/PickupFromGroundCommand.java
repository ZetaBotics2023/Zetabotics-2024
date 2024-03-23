package frc.robot.commands.IntakeCommands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem.IntakeSensorSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.PivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem.REVPivotSubsystem;
import frc.robot.subsystems.LEDSubsystem.CTRELEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDSubsystem.RGBColor;

/**
 * Command to retrieve a note from the ground
 */
public class PickupFromGroundCommand extends Command {

    private IntakeSubsystem intakeSubsystem;
    private PivotSubsystem pivotSubsystem;
    private IntakeSensorSubsystem intakeSensorSubsystem;
    private CTRELEDSubsystem m_ledSubsystem;

    public PickupFromGroundCommand(IntakeSubsystem intakeSusbsystem, PivotSubsystem pivotSubsystem, IntakeSensorSubsystem intakeSensorSubsystem, CTRELEDSubsystem m_ledSubsystem) {
        this.intakeSubsystem = intakeSusbsystem;
        this.pivotSubsystem = pivotSubsystem;
        this.intakeSensorSubsystem = intakeSensorSubsystem;
        this.m_ledSubsystem = m_ledSubsystem;
        addRequirements(this.intakeSubsystem, this.pivotSubsystem, this.intakeSensorSubsystem, this.m_ledSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() { 
        if(!this.intakeSensorSubsystem.isNoteInIntake()) {
            this.pivotSubsystem.setTargetPositionDegrees(IntakeConstants.kGroundPickupPivotRotationDegrees);
            if(this.intakeSensorSubsystem.isNoteInIntake()) {
                this.m_ledSubsystem.setSolidColor(RGBColor.Green.color);
            } else {
                this.m_ledSubsystem.setSolidColor(RGBColor.Red.color);
            }
        }
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
        if(this.intakeSensorSubsystem.isNoteInIntake()) {
            this.m_ledSubsystem.setSolidColor(RGBColor.Green.color);
        } else {
            this.m_ledSubsystem.setSolidColor(RGBColor.Blue.color);
        }
    }

    /**
     * If our sensor detects a note in our intake, stop the command.
     */
    @Override
    public boolean isFinished() {
        return this.intakeSensorSubsystem.isNoteInIntake();
    }
}
