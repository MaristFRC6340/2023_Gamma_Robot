// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// For Autonomous - Sets Position Reference for Wrist

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class WristSetPositionCommand extends CommandBase {

  private WristSubsystem wrist;
  private double newRotation;

  private long startTime;
  private long endTime;
  private double duration = 1; // default 1 second

  /** Creates a new WristSetPositionCommand. */
  public WristSetPositionCommand(WristSubsystem wrist, double rotation, double duration) {
    this.wrist = wrist;
    newRotation = rotation;
    this.duration = duration;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);
  }

  public WristSetPositionCommand(WristSubsystem wrist, double rotation) {
    this.wrist = wrist;
    newRotation = rotation;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    endTime = startTime + (long)(duration * 1000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.setWristReference(newRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    long currentTime = System.currentTimeMillis();
    if (currentTime < endTime) {
      return false;
    }
    else {
      return true;
    }
  }
}