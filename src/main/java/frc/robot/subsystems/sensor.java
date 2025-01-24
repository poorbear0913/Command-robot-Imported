package frc.robot.subsystems;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.lang.Character.Subset;

import edu.wpi.first.wpilibj.DigitalInput;

public class sensor extends SubsystemBase {
    private final DigitalInput photoSensor = new DigitalInput(0);
    // private final 

    @Override
    public void periodic() {
        if (photoSensor.get()) {
            System.out.println("false");  // 正確的大小寫

        } else {
            System.out.println("true");  // 忘記分號的問題已修正
        }
    }
}