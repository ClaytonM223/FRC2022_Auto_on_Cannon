// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;





public class DriveTrain extends SubsystemBase {

  public final WPI_TalonFX rightFront = new WPI_TalonFX(Constants.RIGHT_FRONT_MOTOR);
  public final WPI_TalonFX rightBack = new WPI_TalonFX(Constants.RIGHT_BACK_MOTOR);
  public final WPI_TalonFX leftFront = new WPI_TalonFX(Constants.LEFT_FRONT_MOTOR);
  public final WPI_TalonFX leftBack = new WPI_TalonFX(Constants.LEFT_BACK_MOTOR);
  
  

  public DifferentialDrive drive = new DifferentialDrive(leftFront, rightFront);

  public DriveTrain() {
    
    rightFront.setInverted(true);
    rightBack.setInverted(true); 
    rightBack.follow(rightFront);
    leftBack.follow(leftFront);
    rightFront.setSelectedSensorPosition(0);
    leftFront.setSelectedSensorPosition(0);
    SmartDashboard.putNumber("Right front motor temperature", rightFront.getTemperature());
    SmartDashboard.putNumber("Right back motor temperature", rightBack.getTemperature());
    SmartDashboard.putNumber("Left front motor temperature", leftFront.getTemperature());
    SmartDashboard.putNumber("Left back motor temperature", leftBack.getTemperature());
    drive.feed();
    //drive.setSafetyEnabled(false);
  }
  public void tankDrive (double left , double right){
    drive.tankDrive(left , right);
  }

  public void manualDrive(double Move, double Turn){
    drive.arcadeDrive(Move, Turn);
    if (Math.abs(Move) < 0.025){
      Move = 0;
    }
    if (Math.abs(Turn) < 0.025){
      Turn = 0;
    }
    drive.feed();
    
  } 

  @Override
  public void periodic() {
    double m_rFEncoder = rightFront.getSelectedSensorPosition(0);
    double m_lFEncoder = leftFront.getSelectedSensorPosition(0);
    SmartDashboard.putNumber("Right front encoder?", m_rFEncoder);
    SmartDashboard.putNumber("Left front encoder?", m_lFEncoder);
    SmartDashboard.putNumber("Right Rotations", m_rFEncoder/Constants.TICKS_PER_ROTATION);
    SmartDashboard.putNumber("left Rotations", m_lFEncoder/Constants.TICKS_PER_ROTATION);
    drive.feed();
    // This method will be called once per scheduler run
  }
}
