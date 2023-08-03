package frc.robot.auto;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TimedDriveCommand;
import frc.robot.subsystems.DriveSubsystem;

public class DriveStraightCommandGroup extends SequentialCommandGroup {
    public DriveStraightCommandGroup(DriveSubsystem drive) {

        addCommands(new TimedDriveCommand(drive, 2.7, .75, 0));
    }
}