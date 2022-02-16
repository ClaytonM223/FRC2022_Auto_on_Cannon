// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class PIDturn extends CommandBase {
  
  final double motorMinSpeed = 0.01;
  final double kP = 0.05;
  final double kI = 0.5;
  final double kD = 0.1;
  final double iLimit = 0.5;
  final double wheelCircumference = 8 * Math.PI;
  final double wheelCircumferneceFT = wheelCircumference/12;
  final double WheelrotationsPerFoot = 1/wheelCircumferneceFT;
  final double ticksPerRotation = 2048*12.75;
  final double ticksPerFoot = ticksPerRotation*WheelrotationsPerFoot;
  final double turnCircumferenceIN = 23.5 * Math.PI;
  final double turnCircumferenceFT = turnCircumferenceIN/12;
  final double feetPerDegree = turnCircumferenceFT/360;
  final double ticksPerDegree = ticksPerFoot*feetPerDegree;
  double leftTargetError = 0;
  double rightTargetError = 0;
  double lastTimestamp = 0;
  double lastError = 0;
  double m_turn = 0;

  private final Timer m_timer = new Timer();
  
  /** Creates a new PIDturn. */
  public PIDturn(double turn) {
    m_turn = turn;
    addRequirements(RobotContainer.driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.driveTrain.rightFront.setSelectedSensorPosition(0);
    RobotContainer.driveTrain.leftFront.setSelectedSensorPosition(0);
    m_timer.stop();
    m_timer.reset();
    m_timer.start();
    RobotContainer.driveTrain.rightFront.configOpenloopRamp(1);
    RobotContainer.driveTrain.rightBack.configOpenloopRamp(1);
    RobotContainer.driveTrain.leftFront.configOpenloopRamp(1);
    RobotContainer.driveTrain.leftBack.configOpenloopRamp(1);
    RobotContainer.driveTrain.leftBack.setNeutralMode(NeutralMode.Brake);
    RobotContainer.driveTrain.leftFront.setNeutralMode(NeutralMode.Brake);
    RobotContainer.driveTrain.rightBack.setNeutralMode(NeutralMode.Brake);
    RobotContainer.driveTrain.rightFront.setNeutralMode(NeutralMode.Brake);
    leftTargetError = 0;
    rightTargetError = 0;
    lastTimestamp = Timer.getFPGATimestamp();
 

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Start PID things
    // Calculations

    double m_rfEncoder = RobotContainer.driveTrain.rightFront.getSelectedSensorPosition(0);
    double m_lfEncoder = RobotContainer.driveTrain.leftFront.getSelectedSensorPosition(0);
    double rightError = m_turn*ticksPerDegree - m_rfEncoder;
    double leftError = m_turn*ticksPerDegree - m_lfEncoder;
    double dT = Timer.getFPGATimestamp() - lastTimestamp;
    if (Math.abs(rightError) < iLimit){
       rightTargetError += rightError * dT;   
    }
    if (Math.abs(leftError) < iLimit){
       leftTargetError += leftError * dT;   
    }
    double leftErrorRate = (leftError - lastError) / dT;
    double rightErrorRate = (rightError - lastError) / dT;
    double leftOutputSpeed = kP * leftError + kI * leftTargetError + kD * leftErrorRate;
    double rightOutputSpeed = kP * rightError + kI * rightTargetError + kD * rightErrorRate; 

    if (leftOutputSpeed < motorMinSpeed){
      leftOutputSpeed = 0;
    }
    if (rightOutputSpeed < motorMinSpeed){
      rightOutputSpeed = 0;
    }


    // output to motors
    RobotContainer.driveTrain.rightFront.set(rightOutputSpeed);
    RobotContainer.driveTrain.leftFront.set(leftOutputSpeed);

    // reset all last variables
    lastTimestamp = Timer.getFPGATimestamp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
