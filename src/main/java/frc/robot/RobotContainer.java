// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.EStop;
import frc.robot.commands.Horn_OFF;
import frc.robot.commands.Horn_ON;
import frc.robot.commands.PIDdriveStraight;
import frc.robot.commands.PIDturn;
import frc.robot.commands.Tube1_OFF;
import frc.robot.commands.Tube1_ON;
import frc.robot.commands.Tube2_OFF;
import frc.robot.commands.Tube2_ON;
import frc.robot.commands.Tube3_OFF;
import frc.robot.commands.Tube3_ON;
import frc.robot.commands.Tube4_OFF;
import frc.robot.commands.Tube4_ON;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Valves;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static Valves valves = new Valves();
  public static DriveTrain driveTrain = new DriveTrain();
  public static EStop m_eStop = new EStop();
  public static Horn_ON horn_ON = new Horn_ON();
  public static Horn_OFF horn_OFF = new Horn_OFF();
  public static Tube1_ON tube1_ON = new Tube1_ON();
  public static Tube1_OFF tube1_OFF = new Tube1_OFF();
  public static Tube2_ON tube2_ON = new Tube2_ON();
  public static Tube3_ON tube3_ON = new Tube3_ON();
  public static Tube3_OFF tube3_OFF = new Tube3_OFF();
  public static Tube4_ON tube4_ON = new Tube4_ON();
  public static Tube4_OFF tube4_OFF = new Tube4_OFF();
  public static PIDdriveStraight pidDriveStraight = new PIDdriveStraight();
  public static PIDturn pidTurn = new PIDturn(0);

  public static final Joystick joystick = new Joystick(Constants.JOYSTICK_ID);
  public static final JoystickButton tube1button = new JoystickButton(joystick, Constants.TUBE_1_BUTTON_ID);
  public static final JoystickButton tube2button = new JoystickButton(joystick, Constants.TUBE_2_BUTTON_ID);
  public static final JoystickButton tube3button = new JoystickButton(joystick, Constants.TUBE_3_BUTTON_ID);
  public static final JoystickButton tube4button = new JoystickButton(joystick, Constants.TUBE_4_BUTTON_ID);
  public static final JoystickButton hornbutton = new JoystickButton(joystick, Constants.HORN_BUTTON_ID);
  public static final JoystickButton eStop = new JoystickButton(joystick, Constants.E_STOP);
  
  public double GetJoystickRawAxis(final int axis){
    return joystick.getRawAxis(axis);
  }
 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    tube1button.whenPressed(new Tube1_ON());
    tube1button.whenReleased(new Tube1_OFF());
    tube2button.whenPressed(new Tube2_ON());
    tube2button.whenReleased(new Tube2_OFF());
    tube3button.whenPressed(new Tube3_ON());
    tube3button.whenInactive(new Tube3_OFF());
    tube4button.whenPressed(new Tube4_ON());
    tube4button.whenReleased(new Tube4_OFF());
    hornbutton.whenPressed(new Horn_ON());
    hornbutton.whenReleased(new Horn_OFF());
    eStop.whenPressed(new EStop());
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return 
    new SequentialCommandGroup( 
      new PIDdriveStraight(),
      new PIDturn(180)
    
    );


    
    // An ExampleCommand will run in autonomous  
  }
}
