package frc.robot.auto;


import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GyroDriveCommand;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceCommandGroup extends SequentialCommandGroup {
    public BalanceCommandGroup(DriveSubsystem drive, ADXRS450_Gyro gyro) {
        addCommands(new GyroDriveCommand(drive, gyro, 1.4, 0.4));
        addCommands(new GyroDriveCommand(drive, gyro, 2, 0.15));
    }
}