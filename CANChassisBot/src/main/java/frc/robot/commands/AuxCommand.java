// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AuxSubsystem;

public class AuxCommand extends CommandBase {
  private AuxSubsystem m_aux;
  private double m_speed;
  /** Creates a new AuxCommand. */
  public AuxCommand(AuxSubsystem aux, double speed) {
    m_aux = aux;
    m_speed = speed;
    addRequirements(aux);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_aux.run(m_speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_aux.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
