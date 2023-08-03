package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;


public class BalanceCommand extends CommandBase {
    DriveSubsystem global_drive;
    AHRS global_gyro;
    double global_speed;
    boolean isFinished = false;
    Timer m_timer;

    PIDController m_pid;

    public BalanceCommand(DriveSubsystem drive, AHRS navX, double speed) {
        addRequirements(drive);
        global_drive = drive;
        global_gyro = navX;
        global_speed = speed;

        m_timer = new Timer();

        m_pid = new PIDController(0.016, 0.0001, 0.0);
        m_pid.setTolerance(2);
    }

    @Override
    public void initialize() {
//        global_drive.arcadeDrive(global_speed,0);

    }

    @Override
    public void execute() {
        double degrees = global_gyro.getPitch();
//        double correction = global_speed * (degrees > 0 ? 1 : -1);

        double correction = m_pid.calculate(degrees, 0.0);
        global_drive.arcadeDrive(-correction,0);

        if (m_pid.atSetpoint()) {
            m_timer.reset();
            m_timer.start();
        } else {
            m_timer.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() >= 2;
    }

    @Override
    public void end(boolean interrupted) {
        global_drive.arcadeDrive(0,0);
    }
}
