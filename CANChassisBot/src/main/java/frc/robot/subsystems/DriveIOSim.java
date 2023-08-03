// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import com.revrobotics.SimableCANSparkMax;
import org.snobotv2.module_wrappers.navx.NavxWrapper;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.DifferentialDrivetrainSimWrapper;

public class DriveIOSim implements DriveIO {
    private SimableCANSparkMax m_left, m_right;
    private DifferentialDrivetrainSimWrapper m_sim;
    private AHRS m_gyro;

    /** Subsystem constructor. */
    public DriveIOSim() {
        m_left = new SimableCANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_right = new SimableCANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);

        m_gyro = new AHRS(SPI.Port.kMXP);

        DifferentialDrivetrainSim sim = DifferentialDrivetrainSim.createKitbotSim(
                DifferentialDrivetrainSim.KitbotMotor.kDoubleNEOPerSide,
                DifferentialDrivetrainSim.KitbotGearing.k5p95,
                DifferentialDrivetrainSim.KitbotWheelSize.kSixInch,
                null
        );

        m_sim = new DifferentialDrivetrainSimWrapper(
                sim,
                new RevMotorControllerSimWrapper(m_left),
                new RevMotorControllerSimWrapper(m_right),
                RevEncoderSimWrapper.create(m_left),
                RevEncoderSimWrapper.create(m_right),
                new NavxWrapper().getYawGyro()
        );

        m_sim.setRightInverted(true);
    }

    @Override
    public void setWheelVoltage(double left, double right) {
        m_left.setVoltage(left);
        m_right.setVoltage(right);
    }

    /** Sets speeds to the drivetrain motors. */
    @Override
    public void setWheelSpeeds(double leftSpeedMPS, double rightSpeedMPS) {
        double leftRate = m_left.getEncoder().getVelocity() * (DifferentialDrivetrainSim.KitbotWheelSize.kSixInch.value / 2.0) * DifferentialDrivetrainSim.KitbotGearing.k5p95.value;
        double rightRate = m_right.getEncoder().getVelocity() * (DifferentialDrivetrainSim.KitbotWheelSize.kSixInch.value / 2.0) * DifferentialDrivetrainSim.KitbotGearing.k5p95.value;

        m_left.setVoltage((leftSpeedMPS - leftRate) * 0.1);
        m_right.setVoltage((rightSpeedMPS - rightRate) * 0.1);
    }


    /** Resets robot odometry. */
    @Override
    public void resetPose(Pose2d pose) {
        m_sim.resetOdometry(pose);
    }

    /** Check the current robot pose. */
    @Override
    public Pose2d getPose() {
        return m_sim.getSimPose();
    }

    /** Update our simulation. This should be run every robot loop in simulation. */
    @Override
    public void update(DriveIOInputsAutoLogged inputs) {
        m_sim.update();

        inputs.robotHeadingDegrees = m_gyro.getAngle();
        inputs.leftWheelVoltage = m_left.getAppliedOutput() * 12.0;
        inputs.rightWheelVoltage = m_right.getAppliedOutput() * 12.0;
    }
}
