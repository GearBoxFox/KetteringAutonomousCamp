package frc.robot.auto;


import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GyroDriveCommand;
import frc.robot.commands.GyroTurnCommand;
import frc.robot.subsystems.DriveSubsystem;

public class CircleCommandGroup extends SequentialCommandGroup {
    public CircleCommandGroup(DriveSubsystem drive, ADXRS450_Gyro gyro) {
        addCommands(new GyroDriveCommand(drive, gyro, 3, 0.5));
        addCommands(new GyroTurnCommand(drive,gyro,-45));
        addCommands(new GyroDriveCommand(drive, gyro, 1.45, 0.5));
        addCommands(new GyroTurnCommand(drive, gyro,43));
        addCommands(new GyroDriveCommand(drive, gyro, 4,0.5));
        addCommands(new GyroTurnCommand(drive,gyro,-90));
        addCommands(new GyroDriveCommand(drive, gyro, 2.2, 0.5));
        addCommands(new GyroTurnCommand(drive,gyro,-90));
        addCommands(new GyroDriveCommand(drive, gyro, 1.9, 0.5));
        addCommands(new GyroTurnCommand(drive,gyro,-45));
        addCommands(new GyroDriveCommand(drive, gyro, 5.5,0.5));
        addCommands(new GyroTurnCommand(drive, gyro,43));
        addCommands(new GyroDriveCommand(drive, gyro, 1.5,0.5));
        addCommands(new GyroDriveCommand(drive, gyro, 3,0.15));
    }
}