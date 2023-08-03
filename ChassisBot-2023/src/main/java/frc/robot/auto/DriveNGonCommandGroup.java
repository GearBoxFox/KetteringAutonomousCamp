package frc.robot.auto;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.TimedDriveCommand;
import frc.robot.subsystems.DriveSubsystem;

public class DriveNGonCommandGroup extends SequentialCommandGroup {
    public DriveNGonCommandGroup(DriveSubsystem drive, int n) {
        for(int i = 0; i<n; i++){
            addCommands(new TimedDriveCommand(drive,1.0,0.25,0));
            double angle = 360.0/n;
            double speed = 0.25;
            double time = (angle / 320) / speed;
            addCommands(new TimedDriveCommand(drive, time, 0, speed));
        }
    }
}