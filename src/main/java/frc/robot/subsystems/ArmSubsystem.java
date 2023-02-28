// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Intake Subsystem to control Grasper
 * This is a stub class to be completed for Alpha Robot Testing
 * Use 2023Pneumatics2017Robot project for example
 * This system will use Pneumatics for Lifting of Arm and closing of Grasper
 * Thus, this will be for Arm and Intake
 * Angle of Intake is also controlled in this subsystem.
 * 
 * Might consider placing intake into a seperate subsystem.
 */

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.motorcontrol.Spark;


public class ArmSubsystem extends SubsystemBase {

  // Add Solenoids for Arm and Grasper
    private DoubleSolenoid armSol, intakeSolLeft, intakeSolRight;

    private CANSparkMax armLengthMotor;

    private Compressor compressor;
    

  /** Creates a new Intake. */
  public ArmSubsystem() {
    // Initialize Solenoids
    armSol = new DoubleSolenoid(14, PneumaticsModuleType.CTREPCM,0,1);
    intakeSolLeft = new DoubleSolenoid(14, PneumaticsModuleType.CTREPCM, 3, 4);
    intakeSolRight = new DoubleSolenoid(14, PneumaticsModuleType.CTREPCM, 5, 6);
    armLengthMotor = new CANSparkMax(10, MotorType.kBrushed);

    compressor = new Compressor(14, PneumaticsModuleType.CTREPCM);
    compressor.enableDigital();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  // Add Methods to Control Intake and Arm Solenoids
  public void armUp(){
    armSol.set(Value.kForward);
  }

  public void armDown(){
    armSol.set(Value.kReverse);
  }

  public void openLeft(){
    intakeSolLeft.set(Value.kForward);
  }

  public void closeLeft(){
    intakeSolLeft.set(Value.kReverse);
  }

  public void openRight(){
    intakeSolRight.set(Value.kForward);
  }

  public void closeRight(){
    intakeSolRight.set(Value.kReverse);
  }

  public void setArmLengthMotorPower(double power){
    armLengthMotor.set(power);
  }

  // added by Olivia 2.23.23
  public void setWristMotorPower(double power) {

  }

  public void setIntakeLeftMotorPower(double power) {

  }

  public void turnCompressorOn() {
    compressor.enableDigital();
  }

  public void turnCompressorOff() {
    compressor.disable();
  }

  public CANSparkMax getWristMotor() {
    return null;
  }

}
