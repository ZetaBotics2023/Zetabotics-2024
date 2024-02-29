package frc.robot.utils;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutoCommands.WaitCommandWrapper;

public class BatteryCharge {
    private static double[] batteryVoltages = new double[40];
    private static int nextUpdateIndex = 0;
    private static WaitCommandWrapper poleTimer;
    private static double poleTimeSeconds = 20;
    private static PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
    private static double nextPoleTime = Timer.getFPGATimestamp();

    public static void updateBatteryVoltage() {  
        SmartDashboard.putBoolean("Pole Time", nextPoleTime <= Timer.getFPGATimestamp());
        if(nextPoleTime <= Timer.getFPGATimestamp()) {
            nextPoleTime = Timer.getFPGATimestamp() + poleTimeSeconds;
            if(nextUpdateIndex > batteryVoltages.length-1) {
                nextUpdateIndex = 0;
            }
            batteryVoltages[nextUpdateIndex] = pdh.getVoltage();
            
            nextUpdateIndex++;
            poleTimer = null;
        }
    }

    public static double getAverageVoltage() {
        double totalVoltage = 0;
        for(int i = 0; i < batteryVoltages.length; i++) {
            totalVoltage += batteryVoltages[i];
        }
        return totalVoltage / batteryVoltages.length;
    }
}
