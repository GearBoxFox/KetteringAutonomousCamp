// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AuxSubsystem extends SubsystemBase {

  private VictorSPX aux;

  /** Creates a new Aux1Subsystem. */
  public AuxSubsystem(int port) {
    aux = new VictorSPX(port);
  }

  public void run(double power) {
    aux.set(ControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
