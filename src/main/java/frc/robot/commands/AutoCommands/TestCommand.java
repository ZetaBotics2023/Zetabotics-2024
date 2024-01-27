package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem.IntakeSensorSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.PivotSubsystem;

public class TestCommand extends Command {
    double endTime;
    public TestCommand() {
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.endTime = Timer.getFPGATimestamp() + 2;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Ran Command", true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() > this.endTime;
    }
}
