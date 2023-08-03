// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;

public class DriveSubsystem extends SubsystemBase {
  DriveIO m_io;
  Field2d m_field;
  DriveIOInputsAutoLogged m_inputs;

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem(DriveIO io) {
    m_io = io;
    m_field = new Field2d();
    SmartDashboard.putData("Field", m_field);

    m_inputs = new DriveIOInputsAutoLogged();
  }

  public void drive(double forwardSpeed, double turnSpeed) {
    double left = forwardSpeed + turnSpeed;
    double right = forwardSpeed - turnSpeed;

    m_io.setWheelVoltage(left * 12.0, right * 12.0);
  }

  public RamseteAutoBuilder getAutoBuilder(HashMap<String, Command> eventMap) {
    RamseteController ramset = new RamseteController();

    return new RamseteAutoBuilder(
            m_io::getPose,
            m_io::resetPose,
            ramset,
            new DifferentialDriveKinematics(Units.inchesToMeters(13)),
            m_io::setWheelVoltage,
            eventMap,
            true
      );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_io.update(m_inputs);
    Logger.getInstance().processInputs("Drive", m_inputs);
    m_field.setRobotPose(m_io.getPose());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_io.update(m_inputs);
    m_field.setRobotPose(m_io.getPose());
  }
}
