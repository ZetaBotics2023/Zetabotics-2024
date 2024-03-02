package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystem;


public class RampShooterAtDifforentSpeedCommandCenter extends Command {
    private ShooterSubsystem shooterSubsystem;

    public RampShooterAtDifforentSpeedCommandCenter(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        SmartDashboard.putNumber("Shooter RPM", SmartDashboard.getNumber("Shooter RPM", 4200));
        //ShooterConstants.kShooterRPM = SmartDashboard.getNumber("Shooter RPM", 4200);
        this.shooterSubsystem.runAtRPMAndRPMRatio(2650);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return this.shooterSubsystem.isLeftMotorAtTargetVelocity() && this.shooterSubsystem.isRightMotorAtTargetRatioVelocity();
    }
}
