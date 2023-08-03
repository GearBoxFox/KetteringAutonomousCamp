// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.AvalButtons;
import frc.robot.subsystems.AuxSubsystem;

public class AuxCommand extends CommandBase {
  private AuxSubsystem m_aux;
  private Joystick m_js0, m_js1;
  private String m_tabName;
  private ShuffleboardTab tab;

  private GenericEntry m_invert, m_max, m_jsSel;
  private SendableChooser<Constants.AvalButtons> m_buttons;

  private boolean currInvert = false;

  /** Creates a new AuxCommand. */
  public AuxCommand(AuxSubsystem aux, Joystick js0, Joystick js1, String tabName) {
    // Setup globals
    m_aux = aux;
    m_js0 = js0;
    m_js1 = js1;
    m_tabName = tabName;

    // Add requirements
    addRequirements(aux);

    // Get Shffleboard Tab
    tab = Shuffleboard.getTab(m_tabName);

    // Setup shuffleboard widgets
    m_invert = tab.add("Invert", currInvert)
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .withPosition(0, 2)
        .getEntry();

    m_jsSel = tab.add("Use Aux Stick", false)
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .withPosition(1, 2)
        .getEntry();

    m_max = tab.add("Max Speed", 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1))
        .withPosition(0, 1)
        .withSize(2, 1)
        .getEntry();

    m_buttons = new SendableChooser<Constants.AvalButtons>();

    // Register all button combos
    m_buttons.setDefaultOption(AvalButtons.A_Y.label, AvalButtons.A_Y);
    m_buttons.addOption(AvalButtons.Triggers.label, AvalButtons.Triggers);
    m_buttons.addOption(AvalButtons.X_B.label, AvalButtons.X_B);
    m_buttons.addOption(AvalButtons.Back_Start.label, AvalButtons.Back_Start);
    m_buttons.addOption(AvalButtons.Bumpers.label, AvalButtons.Bumpers);

    tab.add("Buttons", m_buttons)
        .withPosition(0, 0)
        .withSize(2, 1);
  }

  public void setButtons(AvalButtons buttonGroup) {
    m_buttons.setDefaultOption("A/Y", buttonGroup);
  }

  public void setInverted(boolean inverted) {
     m_invert.setBoolean(inverted);
  }

  public void swapInverted() {
    m_invert.setBoolean(!m_invert.getBoolean(true));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Read from suffleboard
    AvalButtons selBut = m_buttons.getSelected();    
    currInvert = m_invert.getBoolean(currInvert);
    double max = m_max.getDouble(1);
    boolean useAux = m_jsSel.getBoolean(false);

    m_aux.run(calcSpeed(currInvert, max, selBut, useAux));
  }

  public double calcSpeed(boolean invert, double max, AvalButtons button, boolean useAux) {
    // Determine which JS to use
    Joystick js = useAux ? m_js1 : m_js0;

    // A/Y = 0
    if (button != AvalButtons.Triggers) {
      // Calculate what button to look for
      int forwardButton = (button == AvalButtons.A_Y) ? Constants.JS_A
          : (button == AvalButtons.X_B) ? Constants.JS_X : (button == AvalButtons.Back_Start) ? Constants.JS_BACK : Constants.JS_LB;
      int reverseButton = (button == AvalButtons.A_Y) ? Constants.JS_Y
          : (button == AvalButtons.X_B) ? Constants.JS_B : (button == AvalButtons.Back_Start) ? Constants.JS_START : Constants.JS_RB;

      // If forward button is pressed
      if (js.getRawButton(forwardButton)) {
        return (invert ? -1 : 1) * max;
      } else if (js.getRawButton(reverseButton)) {
        return (invert ? 1 : -1) * max;
      } else {
        return 0;
      }
    } else { // Triggers = 1
      return (js.getRawAxis(Constants.LEFT_TRIGGER) - js.getRawAxis(Constants.RIGHT_TRIGGER)) * max
          * (invert ? -1 : 1);
    }
  }

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
