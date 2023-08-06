package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class DriveIOSparkMax implements DriveIO {
    CANSparkMax m_leftMaster, m_leftFollower, m_rightMaster, m_rightFollower;
    private DifferentialDriveOdometry m_odometry;
    private AHRS m_gyro;

    public DriveIOSparkMax() {
        // Instantiate Motors
        m_rightFollower = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_rightMaster = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_leftFollower = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_leftMaster = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);

        // Set Reverse
//        m_rightMaster.setInverted(true);
//        m_rightFollower.setInverted(true);

        m_rightFollower.follow(m_rightMaster);
        m_leftFollower.follow(m_leftMaster);

//        m_leftMaster.burnFlash();
//        m_leftFollower.burnFlash();
//        m_rightMaster.burnFlash();
//        m_rightFollower.burnFlash();

        m_gyro = new AHRS(SPI.Port.kMXP);

        m_gyro.calibrate();
        m_gyro.reset();

        m_odometry = new DifferentialDriveOdometry(new Rotation2d(),
                getLeftEncoderDistance(),
                getRightEncoderDistance(),
                new Pose2d());

        resetPose(new Pose2d());
    }

    @Override
    public void setWheelVoltage(double left, double right) {
        m_leftMaster.setVoltage(left);
        m_rightMaster.setVoltage(right);
    }

    @Override
    public void setWheelSpeeds(double leftSpeedMPS, double rightSpeedMPS) {
        double leftFeedforward = m_feedforward.calculate(leftSpeedMPS);
        double rightFeedforward = m_feedforward.calculate(rightSpeedMPS);
        double leftOutput =
                m_leftPIDController.calculate(m_leftMaster.getEncoder().getVelocity() * , speeds.leftMetersPerSecond);
        double rightOutput =
                m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);

        m_leftGroup.setVoltage(leftOutput + leftFeedforward);
        m_rightGroup.setVoltage(rightOutput + rightFeedforward);
    }

    @Override
    public void update(DriveIOInputsAutoLogged inputs) {
        inputs.leftWheelVoltage = m_leftMaster.getAppliedOutput() * 12.0;
        inputs.rightWheelVoltage = m_rightMaster.getAppliedOutput() * 12.0;
        inputs.robotHeadingDegrees = m_gyro.getRotation2d().getDegrees();

        SmartDashboard.putNumber("Left Wheel Meters", getLeftEncoderDistance());
        SmartDashboard.putNumber("Right Wheel Meters", getRightEncoderDistance());

        m_odometry.update(m_gyro.getRotation2d(), getLeftEncoderDistance(), getRightEncoderDistance());
    }

    @Override
    public void resetPose(Pose2d pose) {
        m_leftMaster.getEncoder().setPosition(0.0);
        m_leftFollower.getEncoder().setPosition(0.0);
        m_rightMaster.getEncoder().setPosition(0.0);
        m_rightFollower.getEncoder().setPosition(0.0);

        m_odometry.resetPosition(pose.getRotation(), 0.0, 0.0, pose);
    }

    @Override
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public double getLeftEncoderDistance() {
        return -m_leftMaster.getEncoder().getPosition() * (DifferentialDrivetrainSim.KitbotWheelSize.kSixInch.value * Math.PI) / DifferentialDrivetrainSim.KitbotGearing.k10p71.value;
    }

    public double getRightEncoderDistance() {
        return m_rightMaster.getEncoder().getPosition() * (DifferentialDrivetrainSim.KitbotWheelSize.kSixInch.value * Math.PI) / DifferentialDrivetrainSim.KitbotGearing.k10p71.value;
    }

}
