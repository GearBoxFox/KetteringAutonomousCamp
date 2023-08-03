package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TimedDriveCommand extends CommandBase{
    DriveSubsystem global_drive;
    double global_totalTime, global_fS, global_tS;
    Timer global_timer;
    public TimedDriveCommand(DriveSubsystem drive, double totalTime, double forwardSpeed, double turnSpeed) {
        addRequirements(drive);
        global_drive = drive;
        global_totalTime = totalTime;
        global_fS = forwardSpeed;
        global_tS = turnSpeed;
        global_timer = new Timer();
    }

    @Override
    public void initialize() {
        global_drive.arcadeDrive(global_fS, global_tS);
        global_timer.reset();
        global_timer.start();


    }

    @Override
    public void execute() {
        global_drive.arcadeDrive(global_fS, global_tS);
    }

    @Override
    public void end(boolean interrupted) {
        global_timer.stop();
        global_drive.arcadeDrive(0,0);
    }

    @Override
    public boolean isFinished() {
        return global_timer.get() >= global_totalTime;
    }
}