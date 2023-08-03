package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;


public class GyroTurnCommand extends CommandBase {
    private final DriveSubsystem m_drive;
    private final ADXRS450_Gyro m_gyro;
    private double angle;
    public GyroTurnCommand(DriveSubsystem driveSubsystem, ADXRS450_Gyro gyro, double angle) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(driveSubsystem);
        m_drive = driveSubsystem;
        m_gyro = gyro;
        this.angle = angle;
    }

    @Override
    public void initialize() {
        m_gyro.reset();
    }

    @Override
    public void execute() {
        double error = angle - Math.abs(m_gyro.getAngle());
        double output = error * -0.15;
        output = MathUtil.clamp(output, -0.3, 0.3);
        m_drive.arcadeDrive(0.0, output);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return Math.abs(angle) - Math.abs(m_gyro.getAngle()) < 3;
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.arcadeDrive(0.0, 0.0);
    }
}
