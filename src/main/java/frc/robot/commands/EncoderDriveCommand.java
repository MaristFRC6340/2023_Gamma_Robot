// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GammaDriveSubsystem;

public class EncoderDriveCommand extends CommandBase {
  //fields
  private double distanceFeet;
  private double power;
  private double target;
  private double error;
  private double kP = 0.01;
  private double kP_Distance = 1;
  private double distanceError;

  private GammaDriveSubsystem drivetrain;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;

  public EncoderDriveCommand (GammaDriveSubsystem dtrain, double tarDistance, double pwr)
  {
      drivetrain = dtrain;
      distanceFeet = tarDistance;
      power = pwr;

      leftEncoder = drivetrain.getLeftEnc();
      rightEncoder = drivetrain.getRightEnc();

      addRequirements(drivetrain);

      drivetrain.setBrakeMode();
  }

  public void initialize()
  {
      leftEncoder.setPosition(0);
      rightEncoder.setPosition(0);
      error = 0;

      //target = distanceFeet * 5.0 * 12.0 / 10; // Temp
      target = distanceFeet / 10;

      drivetrain.setBrakeMode();
  }

  public void execute()
  {
      error = leftEncoder.getPosition() + rightEncoder.getPosition(); // Right goes forward, left goes backward
      distanceError = target - leftEncoder.getPosition();

      //double turnAdjust = error * kP;
      double turnAdjust = 0;

      //double powerAdjust = distanceError * kP_Distance;
      double powerAdjust = power;

      double motorPower = powerAdjust + turnAdjust;
      if (motorPower > power) {
        motorPower = power;
      }
      if (motorPower < -power) {
        motorPower = -power;
      }

      double currentDistance = leftEncoder.getPosition();
      System.out.println(currentDistance + ", " + distanceError + ", " + motorPower);

      drivetrain.drive((motorPower-turnAdjust), (motorPower + turnAdjust));

      /*
      if (target > 0) {
        drivetrain.drive(-(motorPower-turnAdjust), -(motorPower + turnAdjust));
      }
      else {
        drivetrain.drive(motorPower-turnAdjust, motorPower+turnAdjust);
      }
      */
        


  }

  public void end(boolean interrupted)
  {
      drivetrain.drive(0,0);
      leftEncoder.setPosition(0);
      rightEncoder.setPosition(0);
  }

  public boolean isFinished() {
      
      double currentDistance = leftEncoder.getPosition();
      //System.out.println(currentDistance + ", " + distanceError);

      if (Math.abs(distanceError) < 0.05) {
        return true;
      }

      /*
      if (target > 0 && currentDistance > target) {
        return true;
      }
      if (target < 0 && currentDistance < target) {
        return true;
      }
      */

      return false;
  }
  
}