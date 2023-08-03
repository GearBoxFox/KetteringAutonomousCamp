package frc.robot.commands;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;


public class ContinuousDriveCommand extends CommandBase {
    DriveSubsystem global_drive;
    ADXRS450_Gyro global_gyro;
    double global_speed;
    double global_totalTime;
    Timer global_timer;

    public ContinuousDriveCommand(DriveSubsystem drive, ADXRS450_Gyro gyro, double totalTime, double forwordSpeed) {
        addRequirements(drive);
        global_drive = drive;
        global_gyro = gyro;
        global_totalTime = totalTime;
        global_speed = forwordSpeed;
        global_timer = new Timer();
    }

    @Override
    public void initialize() {
            global_timer.reset();
            global_timer.start();
            global_drive.arcadeDrive(global_speed,0);

    }

    @Override
    public void execute() {
        double degrees = global_gyro.getRotation2d().getDegrees();
        double correction = Math.abs(degrees) > 3 ? 0.25 * (degrees/Math.abs(degrees)) : 0;
        global_drive.arcadeDrive(global_speed,correction);

    }

    @Override
    public boolean isFinished() {
        return global_timer.get() >= global_totalTime;
    }

    @Override
    public void end(boolean interrupted) {
        global_timer.stop();
        global_drive.arcadeDrive(0,0);
    }
}
