package frc.robot.auto;


import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.ContinuousDriveCommand;
import frc.robot.commands.GyroDriveCommand;
import frc.robot.commands.GyroTurnCommand;
import frc.robot.subsystems.DriveSubsystem;
import com.kauailabs.navx.frc.AHRS;

public class DoubleBalanceCommandGroup extends SequentialCommandGroup {
    public DoubleBalanceCommandGroup(DriveSubsystem drive, ADXRS450_Gyro gyro, AHRS navX) {
        addCommands(
                new GyroDriveCommand(drive, gyro, 0.9, 0.25),
                new GyroDriveCommand(drive, gyro, 1.2, 0.4),
                new BalanceCommand(drive, navX, 0.25),
                new GyroDriveCommand(drive, gyro, 3, 0),
                new GyroTurnCommand(drive, gyro, -5),
                new GyroDriveCommand(drive, gyro, 3.8, .5),
                new GyroDriveCommand(drive, gyro, 1.2, 0.4),
                new BalanceCommand(drive, navX, 0.25),
                new GyroDriveCommand(drive, gyro, 3, 0),
                new GyroDriveCommand(drive, gyro, 3, .3),
                new GyroDriveCommand(drive, gyro, 2, .1)
        );
    }
}