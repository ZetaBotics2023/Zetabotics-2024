package frc.robot.subsystems.IntakeSubsystem;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSensorSubsystem extends SubsystemBase{
    private TimeOfFlight distenceSensor;

    public IntakeSensorSubsystem() {
        this.distenceSensor =  new TimeOfFlight(IntakeConstants.distenceSensorID);
        this.distenceSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
    }


    public boolean isNoteInIntake() {
        return this.distenceSensor.isRangeValid() ? (this.distenceSensor.getRange()
         < IntakeConstants.farthestNotePositionMilameters) : false;
    }

  

    
}
