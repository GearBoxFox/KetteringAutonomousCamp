package frc.robot.auto;


import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GyroDriveCommand;
import frc.robot.commands.GyroTurnCommand;
import frc.robot.commands.TimedDriveCommand;
import frc.robot.subsystems.DriveSubsystem;

public class GyroNGonCommandGroup extends SequentialCommandGroup {
    public GyroNGonCommandGroup(DriveSubsystem drive, ADXRS450_Gyro gyro, int n) {
        for(int i = 0; i<n; i++){
            addCommands(new GyroDriveCommand(drive,gyro,0.5,0.5));
            double angle = (360.0/n);
            addCommands(new GyroTurnCommand(drive, gyro, angle));
        }
    }
}