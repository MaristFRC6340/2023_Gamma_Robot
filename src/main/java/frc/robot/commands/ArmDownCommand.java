// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmDownCommand extends CommandBase {

  private final ArmSubsystem arm;
  private double duration;

  private long startTime;
  private long endTime;
  /** Creates a new ArmDownCommand. */
  public ArmDownCommand(ArmSubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    duration = 1;
  }

  public ArmDownCommand(ArmSubsystem arm, double duration) {
    this.arm = arm;
    this.duration = duration;
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
    arm.armDown();
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