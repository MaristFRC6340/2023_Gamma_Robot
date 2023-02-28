// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GammaDriveSubsystem;


public class DriveTimeCommand extends CommandBase {

  private GammaDriveSubsystem drive;

  // Fields for Time Control
  private long startTime = 0;
  private long endTime = 0;
  private double duration = 0;

  private double power = 0;
  
  /** Creates a new DriveTimeCommand. */
  public DriveTimeCommand(GammaDriveSubsystem drive, double power, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.power = power;

    this.duration = time;

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
    drive.drive(power, power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(0, 0);
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
