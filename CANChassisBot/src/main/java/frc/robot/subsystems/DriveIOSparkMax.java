package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;

public class DriveIOSparkMax implements DriveIO {
    CANSparkMax m_leftMaster, m_leftFollower, m_rightMaster, m_rightFollower;
    private DifferentialDriveOdometry m_odometry;
    private AHRS m_gyro;

    public DriveIOSparkMax() {
        // Instantiate Motors
        m_rightFollower = new CANSparkMax(Constants.RearRight, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_rightMaster = new CANSparkMax(Constants.FrontRight, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_leftFollower = new CANSparkMax(Constants.RearLeft, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_leftMaster = new CANSparkMax(Constants.FrontLeft, CANSparkMaxLowLevel.MotorType.kBrushless);

        // Set Reverse
        m_rightMaster.setInverted(true);
        m_rightFollower.setInverted(true);

        m_rightFollower.follow(m_rightMaster);
        m_leftFollower.follow(m_leftMaster);

        m_gyro = new AHRS(SPI.Port.kMXP);

        m_odometry = new DifferentialDriveOdometry(new Rotation2d(),
                getLeftEncoderDistance(),
                getRightEncoderDistance(),
                new Pose2d());
    }

    @Override
    public void setWheelVoltage(double left, double right) {
        m_leftMaster.setVoltage(left);
        m_rightMaster.setVoltage(right);
    }

    @Override
    public void setWheelSpeeds(double leftSpeedMPS, double rightSpeedMPS) {

    }

    @Override
    public void update(DriveIOInputsAutoLogged inputs) {
        inputs.leftWheelVoltage = m_leftMaster.getAppliedOutput() * 12.0;
        inputs.rightWheelVoltage = m_rightMaster.getAppliedOutput() * 12.0;
        inputs.robotHeadingDegrees = m_gyro.getRotation2d().getDegrees();

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
        return m_leftMaster.getEncoder().getPosition() * 360 * (1.0 / 3.0);
    }

    public double getRightEncoderDistance() {
        return m_rightMaster.getEncoder().getPosition() * 360 * (1.0 / 3.0);
    }

}
