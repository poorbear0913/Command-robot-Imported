package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Limelight;
import frc.robot.Constants.CANdleCOnstants;

import javax.swing.text.html.HTML.Tag;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

public class Candle extends SubsystemBase{
    private final CANdle candle = new CANdle(CANdleCOnstants.ID);

    public RainbowAnimation RainbowState = new RainbowAnimation(0, 0, 0); // Rainbow
    public LarsonAnimation TagDectected = new LarsonAnimation(225, 0, 225); // Purple  // sends a pocket of light across the LED strip
    public StrobeAnimation TagAligning = new StrobeAnimation(225, 255, 0); // Yellow  // strobes the LEDs a specified color
    public StrobeAnimation TagAligned = new StrobeAnimation(0, 225, 225); // Blue
    public StrobeAnimation Accurate = new StrobeAnimation(0, 255, 0);
    public StrobeAnimation Error = new StrobeAnimation(255, 0, 0);


    public static final double BlinkSpeed = 0.4; 

    public Candle(){
        candle.configFactoryDefault();
        candle.configLEDType(LEDStripType.RGB);

        candle.animate(RainbowState);
    }
    // Rainbow
    public void SetDefault(){
        candle.animate(RainbowState);
        RainbowState.setSpeed(0.4); 
        RainbowState.setNumLed(8); 
    }
    // Purple
    public void TagDetected(){
        candle.animate(TagDectected);
        TagDectected.setSpeed(0.4); 
        TagDectected.setNumLed(8); 
        // TagDectected.setSize(5); 

        candle.setLEDs(225, 0, 225);
    }
    // Yellow
    public void TagAligning(){
        candle.animate(TagAligning);
        TagAligning.setSpeed(BlinkSpeed);
    }
    // Blue
    public void TagAligned(){
        candle.animate(TagAligned);
        TagAligning.setSpeed(BlinkSpeed);
    }

    public void Accurate(){
        candle.animate(Accurate);
    }
    public void Error(){
        candle.animate(Error);
    }
    
    @Override
    public void periodic(){

        // double Fiducial_ID = SmartDashboard.getNumber("Fiducial ID", -1);
        double Fiducial_ID = LimelightHelpers.getFiducialID("");
        double Tag_Area = LimelightHelpers.getTA("");
        double Tag_X = LimelightHelpers.getTX("");

        
        /** 
        if(Fiducial_ID == -1){
            SetDefault();
        }
        else if(Fiducial_ID != -1){
            TagDetected();

            if(Output_X != 0 || Output_Y != 0){
                TagAligning();
            }
            else{
                TagAligned();
            }
        }
       */
        if (Tag_Area == 0){
           SetDefault() ;
        }
        
        else if (Tag_Area != 0){
            TagDetected();
            
            if((Tag_Area > 0) & (Tag_Area < 3)){
                TagAligning();
            }
            else{
                TagAligned();
            }
        }
        else {
            if (Tag_X == 0){
           Accurate() ;
        }
        
        else {
            Error();
        }

        } 
    }
}