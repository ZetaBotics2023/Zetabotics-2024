package frc.robot.commands.ShooterCommands;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem.IntakeSensorSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.PivotSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.IntakeCommands.HandOffToShooterCommand;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystem;


public class RampShooterAtDifforentSpeedCommand extends Command {
    private ShooterSubsystem shooterSubsystem;

    public RampShooterAtDifforentSpeedCommand(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        SmartDashboard.putNumber("Shooter RPM", SmartDashboard.getNumber("Shooter RPM", 4200));
        //ShooterConstants.kShooterRPM = SmartDashboard.getNumber("Shooter RPM", 4200);
        this.shooterSubsystem.runAtRPMAndRPMRatio(ShooterConstants.kShooterRPM);
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
