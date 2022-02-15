// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


//import edu.wpi.first.hal.CTREPCMJNI;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Valves extends SubsystemBase {
  /** Creates a new Valves. */
  public Solenoid tube1 = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.TUBE_1_ID);
  public Solenoid tube2 = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.TUBE_2_ID);
  public Solenoid tube3 = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.TUBE_3_ID);
  public Solenoid tube4 = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.TUBE_4_ID);
  public Solenoid horn = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.HORN_ID);

  public Valves() {
    }
  public void ActuateSolenoid (final boolean ON, final boolean OFF){
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
