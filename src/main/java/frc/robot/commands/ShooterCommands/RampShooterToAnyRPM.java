package frc.robot.commands.ShooterCommands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystem;


public class RampShooterToAnyRPM extends Command {
    private ShooterSubsystem shooterSubsystem;
    private double desiredRPM;
    private double voltage;
    public RampShooterToAnyRPM(ShooterSubsystem shooterSubsystem, double desiredRPM, double voltage){
        this.shooterSubsystem = shooterSubsystem;
        this.desiredRPM = desiredRPM;
        this.voltage = voltage;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        //SmartDashBoard.putNumber("Shooter RPM", //SmartDashBoard.getNumber("Shooter RPM", 4200));
        //ShooterConstants.kShooterRPM = //SmartDashBoard.getNumber("Shooter RPM", 4200);
        this.shooterSubsystem.runAtVoltage(this.desiredRPM, this.voltage);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;//this.shooterSubsystem.isLeftMotorAtTargetVelocity() && this.shooterSubsystem.isRightMotorAtTargetRatioVelocity();
    }
}
