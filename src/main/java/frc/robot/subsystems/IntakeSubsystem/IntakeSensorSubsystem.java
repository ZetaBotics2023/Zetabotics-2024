package frc.robot.subsystems.IntakeSubsystem;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSensorSubsystem extends SubsystemBase{
    private TimeOfFlight distanceSensor;

    public IntakeSensorSubsystem() {
        this.distanceSensor =  new TimeOfFlight(IntakeConstants.distenceSensorID);
        this.distanceSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Distence Sensor Value", this.distanceSensor.getRange());
    }

    public double sensedPosition() {
        return this.distanceSensor.getRange();
    }
    public boolean isNoteInIntake() {
        /* 
        double readingSD = this.distanceSensor.getRangeSigma() * 3;
        double upperReading = this.distanceSensor.getRange() + readingSD;
        double lowerReading = this.distanceSensor.getRange() - readingSD;
        boolean isInUpperRange = upperReading
         < IntakeConstants.farthestNotePositionMilameters;
         
         
        boolean isInLowerRange = lowerReading
         < IntakeConstants.farthestNotePositionMilameters;
         */

        boolean isInRange = this.distanceSensor.getRange() < IntakeConstants.kFarthestNotePositionMilameters;
        return this.distanceSensor.isRangeValid() ? isInRange : false;
    }

  

    
}
