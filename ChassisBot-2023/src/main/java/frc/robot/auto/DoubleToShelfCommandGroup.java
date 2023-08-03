package frc.robot.auto;


import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.GyroDriveCommand;
import frc.robot.commands.GyroTurnCommand;
import frc.robot.subsystems.DriveSubsystem;

public class DoubleToShelfCommandGroup extends SequentialCommandGroup {
    public DoubleToShelfCommandGroup(DriveSubsystem drive, ADXRS450_Gyro gyro) {
        addCommands(new GyroDriveCommand(drive, gyro, 3, 0.5));
        addCommands(new GyroTurnCommand(drive,gyro,-45));
        addCommands(new GyroDriveCommand(drive, gyro, 1.4, 0.5));
        addCommands(new GyroTurnCommand(drive, gyro,43));
        addCommands(new GyroDriveCommand(drive, gyro, 4, 0.5));
        addCommands(new GyroDriveCommand(drive, gyro, 2, 0.25));
    }
}