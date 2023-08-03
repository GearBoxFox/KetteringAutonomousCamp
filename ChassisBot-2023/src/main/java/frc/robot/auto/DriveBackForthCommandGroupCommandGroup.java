package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TimedDriveCommand;
import frc.robot.subsystems.DriveSubsystem;

public class DriveBackForthCommandGroupCommandGroup extends SequentialCommandGroup {

    public DriveBackForthCommandGroupCommandGroup(DriveSubsystem drive) {
        addCommands(new TimedDriveCommand(drive, 1.5, .5, 0));
//        addCommands(drive.turnDegrees(drive, 180,0.5));
        addCommands(new TimedDriveCommand(drive, 0.9, 0.0, 0.5));
        addCommands(new TimedDriveCommand(drive, 1.5, .5, 0));
    }
}

