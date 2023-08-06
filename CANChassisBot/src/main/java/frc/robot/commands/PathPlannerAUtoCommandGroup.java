package frc.robot.commands;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

import java.util.HashMap;

public class PathPlannerAUtoCommandGroup extends SequentialCommandGroup {
    DriveSubsystem m_drive;
    PathPlannerTrajectory m_traj;

    public PathPlannerAUtoCommandGroup(DriveSubsystem drive, String pathname) {
        m_drive = drive;

        m_traj = PathPlanner.loadPath(pathname, new PathConstraints(0.5, 1.0));

        addCommands(new PrintCommand("Starting Auto Builder"));
        addCommands(m_drive.getAutoBuilder(new HashMap<>()).fullAuto(m_traj));
    }
}