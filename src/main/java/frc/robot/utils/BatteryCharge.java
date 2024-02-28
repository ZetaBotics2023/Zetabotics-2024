package frc.robot.utils;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.commands.AutoCommands.WaitCommandWrapper;

public class BatteryCharge {
    private static double[] batteryVoltages = new double[20];
    private static int nextUpdateIndex = 0;
    private static WaitCommandWrapper poleTimer;
    private static double poleTimeSeconds = 1/4;
    private static PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

    public static void updateBatteryVoltage() {  
        if(poleTimer == null) {
            poleTimer = new WaitCommandWrapper(poleTimeSeconds);
            poleTimer.schedule();
        }

        if(poleTimer.isFinished()) {
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
