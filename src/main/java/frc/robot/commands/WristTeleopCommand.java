// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// This is for Teleoperation

package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class WristTeleopCommand extends CommandBase {

private WristSubsystem wrist;

private CANSparkMax m_motor;
private SparkMaxPIDController m_pidController;
private RelativeEncoder m_encoder;

private double rotations = 0;

  /** Creates a new WristTeleopCommand. */
  public WristTeleopCommand(WristSubsystem wrist, double rotations) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.wrist = wrist;
    this.rotations = wrist.getCurrentRotation();

    m_motor = wrist.getWristMotor();

    m_encoder = m_motor.getEncoder();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    System.out.println(rotations);

    // Use Laddered If Statements for setting rotations state
    // If / Else Statements are for Motor States
    // Button: A: Home Positions
    if (Robot.getArmControlJoystick().getRawButton(1)) {
      rotations = 5;
    }

    // Button: B: Home Positions
    if (Robot.getArmControlJoystick().getRawButton(2)) {
      rotations = 10;
    } 

    // Button: X: Shelf Pickup Position
    if (Robot.getArmControlJoystick().getRawButton(3)) {
      rotations = 51;
    }

    // Button: Y: Pickup from Floor
    if (Robot.getArmControlJoystick().getRawButton(4)) {
      rotations = 33;
    }

    // Protect it going under 0
    if (rotations <= 0) {
      rotations = 0;
    }

    double deltaPos = Robot.getArmControlJoystick().getRawAxis(5);
    if (Math.abs(deltaPos) > 0.4) {
      rotations += deltaPos * 0.5;
    }

    //m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    wrist.setWristReference(rotations);
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