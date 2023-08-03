package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
    @AutoLog
    class DriveIOInputs {
        public double leftWheelVoltage = 0.0;
        public double rightWheelVoltage = 0.0;
        public double robotHeadingDegrees = 0.0;
    }

    default public void setWheelVoltage(double left, double right) {}
    default public void setWheelSpeeds(double leftSpeedMPS, double rightSPeedMPS) {};

    default public void update(DriveIOInputsAutoLogged inputs) {}

    default public void resetPose(Pose2d pose){}

    default public Pose2d getPose() {return new Pose2d();}
}
