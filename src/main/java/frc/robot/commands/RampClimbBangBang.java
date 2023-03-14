// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.GammaDriveSubsystem;

public class RampClimbBangBang extends CommandBase {

  private GammaDriveSubsystem m_drive;
  //double pitch = m_drive.
  private int count = 0;
  private int frameCount = 0;
  private double totalDeltaY;
  private double totalDeltaZ;
  private double avgDeltaY;
  private double avgDeltaZ;
  private int arraySize = 10;
  private double deltaYValues [] = new double[arraySize];
  private double deltaZValues [] = new double[arraySize]; 

  private double oldDeltaY = 0;
  private double motorPower;

  private double kP = 0.03;
  
  /** Creates a new RampClimbBangBang. */
  public RampClimbBangBang(GammaDriveSubsystem drive) {

    m_drive = drive;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // double deltaY = Robot.getRioAccell().getY();
    // double deltaZ = Robot.getRioAccell().getZ();
    // deltaYValues[count] = deltaY;
    // deltaZValues[count] = deltaZ;
    
    // totalDeltaY = 0;
    // totalDeltaZ = 0;
    // for (int i = 0; i < deltaYValues.length; i++) {
    //   totalDeltaY += deltaYValues[i];
    //   totalDeltaZ += deltaZValues[i];
    // }

    // avgDeltaY = totalDeltaY / deltaYValues.length;
    // avgDeltaZ = totalDeltaZ / deltaZValues.length;

    // count++;
    // if (count > deltaYValues.length-1) {
    //   count = 0;
    // }
    
    frameCount++;

    // Set Power
    //double theta = Math.atan2(avgDeltaZ, avgDeltaY);
    double theta = m_drive.gyro.getPitch();
    // theta = Math.toDegrees(theta);
    theta += 3;

    // Pure Trig Flip - Flop Method
    if (theta > 0) {
      motorPower = Math.sin(Math.toRadians(theta - 90)) + 1;
    }
    else {
      motorPower = -Math.sin(Math.toRadians(theta - 90)) - 1;
    // motorPower = Math.sin(Math.toRadians(theta));
    }

    // motorPower = (1/100) * Math.abs(theta) * (60*Math.sin(theta-90) + 60);
    


    motorPower *= 3.75; // Scale Amplitude // WORKED

    /* PID and Trig Method
    double m = kP * Math.abs(theta);

    if (m > 0.5) {
      m = 0.5;
    }

    motorPower = Math.sin(Math.toRadians(theta)) * m;
    */

    double cap = 0.15; // WORKED

    if (motorPower > cap) {
      motorPower = cap;
    }
    if (motorPower < -cap) {
      motorPower = -cap;
    }

    //System.out.println(avgDeltaY);
    if (frameCount > 50) {
      m_drive.drive(motorPower, motorPower);
    }    


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    double deltaY = Robot.getRioAccell().getY();
    boolean lurch = false;

    if (Math.abs(deltaY - oldDeltaY) > 0.3) {
        lurch = true;
    }
    System.out.println("!!!THETA!!!   " + m_drive.gyro.getPitch() + "   MOTOR POWER:" + motorPower);
    // oldDeltaY = deltaY;
    // double theta = Math.atan2(avgDeltaZ, avgDeltaY);
    // theta = Math.toDegrees(theta);
    // theta -= 90;

    // System.out.println("DeltaY: " + avgDeltaY + ", " + "DeltaZ: " + avgDeltaZ + ", "  + "Theta: " + theta + ", " + motorPower);



    /* 
    if (Math.abs(theta) < 4) {
      
      if (frameCount > 100) {
        return true;
      }
      else {
        return false;
      }
      
    }
    else {
      return false;
    }
    */
    return false;
    
  }
}
