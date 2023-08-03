// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AuxSubsystem extends SubsystemBase {

  private PWMVictorSPX aux0;

  /** Creates a new Aux1Subsystem. */
  public AuxSubsystem(int port0) {
    aux0 = new PWMVictorSPX(port0);
  }

  public void run(double power) {
    aux0.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
