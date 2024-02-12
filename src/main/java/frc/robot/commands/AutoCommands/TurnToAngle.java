package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;

public class TurnToAngle extends Command {
  
  private DriveSubsystem driveSubsystem;
  private double targetAngle;
  private double initialAngle;
  private double degreesToTurn;
  
  public TurnToAngle(DriveSubsystem driveSubsystem, double degreesToTurn) {
    this.driveSubsystem = driveSubsystem;
    this.degreesToTurn = degreesToTurn;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     initialAngle = driveSubsystem.getHeading();
     driveSubsystem.resetHeadingPid();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.turnToHeading(targetAngle); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveSubsystem.isHeadingPidAtGoal();
  }
}
