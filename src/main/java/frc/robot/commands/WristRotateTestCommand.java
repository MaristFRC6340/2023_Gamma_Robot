// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// This is only for testing. Do not use in competition

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class WristRotateTestCommand extends CommandBase {

  // Add field for ArmSubsystem
  private WristSubsystem wrist;
  private double clicks;

  /** Creates a new WristRotateTestCommad. */
  public WristRotateTestCommand(WristSubsystem wrist, double clicks) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.wrist = wrist;
    this.clicks = clicks;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset Rotations
    wrist.getWristEncoder().setPosition(0);

    //System.out.println("Wrist Command Initialized");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Set Rotate for Arm to 1 rotation
    if (clicks > 0) {
      wrist.setWristMotorPower(.25);
    }
    else {
      wrist.setWristMotorPower(-.25);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.setWristMotorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // Maybe wait for a time to allow arm to get to rotation?
    double currentRotation = wrist.getWristPosition();

    //System.out.println(currentRotation);

    if (clicks > 0 && currentRotation > clicks) {
      return true;
    }
    else if (clicks < 0 && currentRotation < clicks) {
      return true;
    }
    return false;
  }
}