package frc.robot.auto;


import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GyroDriveCommand;
import frc.robot.commands.GyroTurnCommand;
import frc.robot.subsystems.DriveSubsystem;

public class DoubleToBalanceCommandGroup extends SequentialCommandGroup {
    public DoubleToBalanceCommandGroup(DriveSubsystem drive, ADXRS450_Gyro gyro) {
        addCommands(new GyroDriveCommand(drive, gyro, 3, 0.5));
        addCommands(new GyroTurnCommand(drive,gyro,-45));
        addCommands(new GyroDriveCommand(drive, gyro, 3, 0.5));
        addCommands(new GyroTurnCommand(drive,gyro,44));
        addCommands(new GyroDriveCommand(drive, gyro, 1.3, 0.5));
        addCommands(new GyroDriveCommand(drive, gyro, 1.4, 0.4));
        addCommands(new GyroDriveCommand(drive, gyro, 1.6, 0.15));
        addCommands(new GyroDriveCommand(drive, gyro, 0.8, -0.20));
    }
}