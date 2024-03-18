package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem.IntakeSensorSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.PivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem.REVPivotSubsystem;

import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.IntakeCommands.HandOffToShooterCommand;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystem;


public class ShootAtDiffSpeedCommand extends Command {
    private ShooterSubsystem shooterSubsystem;
    private HandOffToShooterCommand handOffToShooterCommand;
    private boolean isFirstRun = false;
    private double finishedRunningTimeStamp = 0.0;


    public ShootAtDiffSpeedCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSusbsystem, PivotSubsystem pivotSubsystem, IntakeSensorSubsystem intakeSensorSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
        this.handOffToShooterCommand = new HandOffToShooterCommand(intakeSusbsystem, pivotSubsystem, intakeSensorSubsystem);
    }

    @Override
    public void initialize(){
        this.shooterSubsystem.runAtRPMAndRPMRatio(ShooterConstants.kShooterRPM);        
    }

    @Override
    public void execute() {
        if(this.shooterSubsystem.isLeftMotorAtTargetVelocity() && this.shooterSubsystem.isRightMotorAtTargetVelocity()) {
            if(this.isFirstRun){
                this.isFirstRun = false;
                this.handOffToShooterCommand.initialize();
            }
            this.handOffToShooterCommand.execute();
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.shooterSubsystem.runAtRPMAndRPMRatio(0);
        this.handOffToShooterCommand.end(interrupted);
        this.isFirstRun = true;
    }

    @Override
    public boolean isFinished() {
        if(this.handOffToShooterCommand.isFinished()) {
            this.finishedRunningTimeStamp = Timer.getFPGATimestamp() + ShooterConstants.kShootTime;
        }
        return Timer.getFPGATimestamp() > this.finishedRunningTimeStamp;
    }
}
