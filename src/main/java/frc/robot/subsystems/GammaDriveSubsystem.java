// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// Limelight Targeting Code is in this file.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.Robot;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class GammaDriveSubsystem extends SubsystemBase {
  private CANSparkMax leftFront;
  private CANSparkMax leftRear;
  private CANSparkMax rightFront;
  private CANSparkMax rightRear;
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_leftEncoder;
  private RelativeEncoder m_rightEncoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  public AHRS gyro;

  /** Creates a new DriveTrain. */
  public GammaDriveSubsystem() {
    gyro = new AHRS(SPI.Port.kMXP);
    
    leftFront = new CANSparkMax(13, MotorType.kBrushless);
    leftRear = new CANSparkMax(5, MotorType.kBrushless);
    rightFront = new CANSparkMax(4, MotorType.kBrushless);
    rightRear = new CANSparkMax(12, MotorType.kBrushless);
    /**
     * The restoreFactoryDefaults method can be used to reset the configuration
     * parameters
     * in the SPARK MAX to their factory default state. If no argument is passed,
     * these
     * parameters will not persist between power cycles
     */
    //leftFront.restoreFactoryDefaults();   // These should be commented out because we setup
    //rightFront.restoreFactoryDefaults();  // the encoders with the CAN system - Mr. Michaud
                                            // Updated on 23 Feb 22

    /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController
     * object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidController = leftFront.getPIDController();

    // Encoder object created to display position values
    m_leftEncoder = leftFront.getEncoder();
    m_rightEncoder = rightFront.getEncoder();

    // Reset Encoders on Initialization
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);

    // PID coefficients
    kP = 0.1;
    kI = 1e-4;
    kD = 1;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

    // set PID coefficients
    // m_pidController.setP(kP);
    // m_pidController.setI(kI);
    // m_pidController.setD(kD);
    // m_pidController.setIZone(kIz);
    // m_pidController.setFF(kFF);
    // m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // // display PID coefficients on SmartDashboard
    // SmartDashboard.putNumber("P Gain", kP);
    // SmartDashboard.putNumber("I Gain", kI);
    // SmartDashboard.putNumber("D Gain", kD);
    // SmartDashboard.putNumber("I Zone", kIz);
    // SmartDashboard.putNumber("Feed Forward", kFF);
    // SmartDashboard.putNumber("Max Output", kMaxOutput);
    // SmartDashboard.putNumber("Min Output", kMinOutput);
    // SmartDashboard.putNumber("Set Rotations", 0);
    //leftRear.follow(leftFront, false);              // These are set to false - Mr. Michaud
    //rightRear.follow(rightFront, false);            // We set motor powers Manually
                                                      // Updated on 23 Feb 22
  //ledMode.setDouble(3);
 }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(double left, double right) {

    // Run Motors with left and right power
    // Either from Limelight or parameters
    leftFront.set(-left);
    leftRear.set(-left);
    rightFront.set(right);
    rightRear.set(right);
  }

  public void driveDistance(double distance) {

  }

  // getters for encoder drive command
  public RelativeEncoder getLeftEnc() {
    return m_leftEncoder;
  }

  public RelativeEncoder getRightEnc() {
    return m_rightEncoder;
  }

  public void setCoastMode() {
    leftFront.setIdleMode(CANSparkMax.IdleMode.kCoast);
    leftRear.setIdleMode(CANSparkMax.IdleMode.kCoast);
    rightFront.setIdleMode(CANSparkMax.IdleMode.kCoast);
    rightRear.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  public void setBrakeMode() {
    leftFront.setIdleMode(CANSparkMax.IdleMode.kBrake);
    leftRear.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightFront.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightRear.setIdleMode(CANSparkMax.IdleMode.kBrake);
  } 

}