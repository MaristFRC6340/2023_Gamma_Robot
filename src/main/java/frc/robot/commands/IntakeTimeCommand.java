// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Use for Competition

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;


public class IntakeTimeCommand extends CommandBase {
  /** Creates a new IntakeTimeCommand. */

  private final ArmSubsystem arm;
  private long startTime = 0;
  private long endTime = 0;
  private double duration = 0;
  private double intakePower = 0;
  
  public IntakeTimeCommand(ArmSubsystem arm, double power, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    intakePower = power;
    duration = time;

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
    arm.setIntakeLeftMotorPower(intakePower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setIntakeLeftMotorPower(0);
  }

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