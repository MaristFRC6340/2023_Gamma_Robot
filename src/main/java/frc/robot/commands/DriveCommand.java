// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GammaDriveSubsystem;

public class DriveCommand extends CommandBase {

  // Field for DriveSubsystem
  private final GammaDriveSubsystem m_robotDrive;


  /** Creates a new DriveCommand. */
  public DriveCommand(GammaDriveSubsystem m_robotDrive2) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_robotDrive = m_robotDrive2;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Updated Drive Command
    double forwardPower = Robot.getDriveControlJoystick().getRawAxis(1);
    double turnPower = Robot.getDriveControlJoystick().getRawAxis(0);
    double leftPower = (forwardPower - turnPower) * Constants.DriveConstants.SpeedMultiplier;
    double rightPower = (forwardPower + turnPower) * Constants.DriveConstants.SpeedMultiplier;
    m_robotDrive.drive(leftPower, rightPower);

    double deltaZ = Robot.getRioAccell().getZ();
    double deltaX = Robot.getRioAccell().getX();
    double deltaY = Robot.getRioAccell().getY();
    //System.out.println("Delta Z: " + deltaZ + ", DeltaX: " + deltaX);
    System.out.println(deltaX + ", " + deltaY + ", " + deltaZ);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Todo: Set motors to stop
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
