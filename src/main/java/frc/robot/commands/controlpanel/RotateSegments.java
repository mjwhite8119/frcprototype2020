package frc.robot.commands.controlpanel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;

/**
 * Rotates the control panel the specified number of segments
 */
public class RotateSegments extends CommandBase {
  // The subsystem the command runs on
  private final ControlPanelSubsystem m_controlPanel;
  private double m_segments;

  public RotateSegments(double segments, ControlPanelSubsystem subsystem) {
    m_controlPanel = subsystem;
    m_segments = segments;
    addRequirements(m_controlPanel);
  }

  @Override
  public void initialize() {
    m_controlPanel.rotateSegments(m_segments);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}