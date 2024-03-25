package frc.robot.commands.ShooterCommands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystem;


public class RampShooterAtDifforentSpeedCommand extends Command {
    private ShooterSubsystem shooterSubsystem;

    public RampShooterAtDifforentSpeedCommand(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        //SmartDashBoard.putNumber("Shooter RPM", //SmartDashBoard.getNumber("Shooter RPM", 4200));
        //ShooterConstants.kShooterRPM = //SmartDashBoard.getNumber("Shooter RPM", 4200);
        this.shooterSubsystem.runAtVoltage(ShooterConstants.kShooterRPM, 6);
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
