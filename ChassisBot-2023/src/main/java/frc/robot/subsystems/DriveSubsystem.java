// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.TimedDriveCommand;
import com.revrobotics.CANSparkMax;

public class DriveSubsystem extends SubsystemBase {

  private CANSparkMax rearRight, frontRight, rearLeft, frontLeft;
  private double m_maxSpeed;

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    // Instantiate Motors
    rearRight = new CANSparkMax(Constants.RearRight + 1, CANSparkMaxLowLevel.MotorType.kBrushed);
    frontRight = new CANSparkMax(Constants.FrontRight + 1, CANSparkMaxLowLevel.MotorType.kBrushed);
    rearLeft = new CANSparkMax(Constants.RearLeft + 1, CANSparkMaxLowLevel.MotorType.kBrushed);
    frontLeft = new CANSparkMax(Constants.FrontLeft + 1, CANSparkMaxLowLevel.MotorType.kBrushed);

    double m_maxSpeed = 360; // deg/sec

    // Set Reverse
    rearRight.setInverted(false);
    frontRight.setInverted(false);

    rearLeft.setInverted(true);
    frontLeft.setInverted(true);

    rearRight.burnFlash();
    frontRight.burnFlash();

    //motor type
    rearLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
    frontLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rearRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
    frontRight.setIdleMode(CANSparkMax.IdleMode.kBrake);

  }

  public void arcadeDrive(double forwardSpeed, double turnSpeed) {
    double left = -forwardSpeed + turnSpeed;
    double right = -forwardSpeed - turnSpeed;

    rearRight.set(right);
    frontRight.set(right);
    rearLeft.set(left);
    frontLeft.set(left);
  }

  public void tankDrive(double lSpeed, double rSpeed) {

    rearRight.set(rSpeed);
    frontRight.set(rSpeed);
    rearLeft.set(lSpeed);
    frontLeft.set(lSpeed);
  }

  public double degreesToSeconds(double degrees, double percentage) {
    return (m_maxSpeed * percentage) / degrees;
  }


  public double degreesToSeconds(double degrees) {
    return degreesToSeconds(degrees, 1);
  }

  public CommandBase turnDegrees(DriveSubsystem drive, double degrees, double speed) {
    double time = degreesToSeconds(degrees, speed);
    return new TimedDriveCommand(drive, time, 0.0, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
