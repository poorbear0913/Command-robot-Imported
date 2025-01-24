// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;  
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.chassis;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final chassis mChassis = new chassis();
  private final Candle mCandle = new Candle();
  private final Limelight mlimelight = new Limelight();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController m_driverController = new XboxController(0); 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    mChassis.setDefaultCommand(
      new RunCommand(()-> mChassis.drive(m_driverController.getLeftX()*0.5,m_driverController.getRightY()*0.5),mChassis));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via theBGH2frˋˋˋㄅㄅ S
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller++++
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  
  private void configureBindings() {
    new JoystickButton(m_driverController, 1).whileTrue(new RunCommand(mChassis::autotarget)).onFalse(new InstantCommand(mChassis::stop));
    // Schedule exampleMethodCommand when the Xbox controller's B button is pressed,  
    // cancelling on release.
    

    
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  }
} 