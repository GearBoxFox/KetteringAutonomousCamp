package frc.robot.auto;


import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GyroDriveCommand;
import frc.robot.commands.GyroTurnCommand;
import frc.robot.subsystems.DriveSubsystem;

public class GyroBackForthCommandGroup extends SequentialCommandGroup {
    public GyroBackForthCommandGroup(DriveSubsystem drive, ADXRS450_Gyro gyro) {
        addCommands(new GyroDriveCommand(drive, gyro, 1.5, 0.5));
        addCommands(new GyroTurnCommand(drive, gyro, 180));
        addCommands(new GyroDriveCommand(drive, gyro, 1.5, 0.5));

    }
}