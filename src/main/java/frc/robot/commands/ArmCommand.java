// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Robot;

public class ArmCommand extends CommandBase {
  /** Creates a new ArmCommand. */

  private final ArmSubsystem arm;
  double leftY = 0, rightY = 0;
  double leftX = 0, rightX = 0;

  public ArmCommand( ArmSubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Controls for Arm
    if(Robot.getArmControlJoystick().getRawButton(5)){
      arm.armUp();
    }
    if(Robot.getArmControlJoystick().getRawButton(3)){
      arm.armDown();
    }

    // Arm Length and Wrist Control
    leftY = Robot.getArmControlJoystick().getRawAxis(1); //arm motor
    
    rightY = Robot.getArmControlJoystick().getRawAxis(5)/4; //wrist motor

    leftX = Robot.getArmControlJoystick().getRawAxis(4);
    
    rightX = Robot.getArmControlJoystick().getRawAxis(0);

    arm.setArmLengthMotorPower(MathUtil.applyDeadband(leftY, 0.06));

    if(Robot.getArmControlJoystick().getRawButton(6)){ // lb
      arm.armUp();
    }
    if(Robot.getArmControlJoystick().getRawButton(4)){ // rb
      arm.armDown();
    }

    if (Robot.getArmControlJoystick().getRawAxis(2) > 0.25) {
      arm.closeLeft();
      arm.closeRight();
    }

    if (Robot.getArmControlJoystick().getRawAxis(3) > 0.25) {
      arm.openLeft();
      arm.openRight();
    }

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
