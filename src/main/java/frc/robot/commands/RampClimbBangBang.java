// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.GammaDriveSubsystem;

public class RampClimbBangBang extends CommandBase {

  private GammaDriveSubsystem m_drive;

  private int count = 0;
  private int frameCount = 0;
  private double totalDeltaY;
  private double avgDeltaY;
  private int arraySize = 25;
  private double deltaYValues [] = new double[arraySize];

  private double oldDeltaY = 0;

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

    double deltaY = Robot.getRioAccell().getY();
    deltaYValues[count] = deltaY;
    
    totalDeltaY = 0;
    for (int i = 0; i < deltaYValues.length; i++) {
      totalDeltaY += deltaYValues[i];
    }

    avgDeltaY = totalDeltaY / deltaYValues.length;
    count++;
    if (count > deltaYValues.length-1) {
      count = 0;
    }
    
    frameCount++;
    //System.out.println(avgDeltaY);
    if (frameCount > 50) {
      m_drive.drive(0.1, 0.1);
    }
    if (frameCount > 150) {
      m_drive.drive(0.075, 0.075);
    }
    if (frameCount > 250) {
      m_drive.drive(0.05, 0.05);
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

    oldDeltaY = deltaY;
    System.out.println(avgDeltaY + ", " + Math.abs(deltaY - oldDeltaY));
    if (avgDeltaY > -0.15) {
      
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
    
  }
}
