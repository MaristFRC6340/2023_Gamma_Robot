// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// This is for testing. Do not use in Competition

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;


public class WristMoveTimeCommand extends CommandBase {

private final ArmSubsystem arm;
long startTime = 0;
long endTime = 0;
double duration;
double power;

  /** Creates a new WristMoveTimeCommand. */
  public WristMoveTimeCommand(double duration, double power, ArmSubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.duration = duration;
    this.power = power;
    this.arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    endTime = startTime + (long)(duration*1000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setWristMotorPower(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setWristMotorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (endTime < System.currentTimeMillis()) {
      return true;
    }
    return false;
  }
}