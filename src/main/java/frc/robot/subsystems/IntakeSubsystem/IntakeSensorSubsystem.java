package frc.robot.subsystems.IntakeSubsystem;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * Provides methods for accessing the distance reported by our time-of-flight sensor.
 * This is for checking whether a note is loaded in our robot.
 */
public class IntakeSensorSubsystem extends SubsystemBase{
    private TimeOfFlight distanceSensor;

    public IntakeSensorSubsystem() {
        this.distanceSensor = new TimeOfFlight(IntakeConstants.distenceSensorID);
        this.distanceSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
    }

    /**
     * Periodic override that outputs the distance sensor value to SmartDashboard.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Distance Sensor Value", this.distanceSensor.getRange());
    }

    /**
     * Accessor for our sensor's detected distance
     * @return The detected distance in millimeters
     */
    public double sensedPosition() {
        return this.distanceSensor.getRange();
    }
    /**
     * Returns whether or not a not is in our intake.
     * This compares the sensed position to a set of min-max constants to
     * evaluate our ring holding status.
     * @return
     */
    public boolean isNoteInIntake() {
        boolean isInRange = this.distanceSensor.getRange() < IntakeConstants.kFarthestNotePositionMillimeters;
        // Only return the value if the sensor is sensing a range at the current time, otherwise false
        return this.distanceSensor.isRangeValid() && isInRange;
    }

  

    
}
