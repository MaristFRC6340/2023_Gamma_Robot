package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {
    private CANSparkMax wristMotor;
    private RelativeEncoder armRelativeEncoder;
    private SparkMaxPIDController m_pidController;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

    private double currentRotations;

    public WristSubsystem() {
        wristMotor = new CANSparkMax(7, MotorType.kBrushless);
        armRelativeEncoder = wristMotor.getEncoder();

        m_pidController = wristMotor.getPIDController();

        kP = 0.5;
        kI = 1e-4;
        kD = 3;
        kIz = 0;
        kFF = 0;
        kMaxOutput = 0.25;
        kMinOutput = -0.25;

        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    }

    public void setWristMotorPower(double power) {
        wristMotor.set(power);
    }

    public void setWristReference(double rotations) {
      currentRotations = rotations;
      m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    }

    public double getCurrentRotation() {
      return currentRotations;
    }

    public double getWristPosition() {
        return armRelativeEncoder.getPosition();
      }

      
    public RelativeEncoder getWristEncoder() {
     return armRelativeEncoder;
    }
      
    public CANSparkMax getWristMotor() {
      return wristMotor;
    }


      
}