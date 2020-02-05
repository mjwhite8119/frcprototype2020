package frc.robot.commands.controlpanel;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.subsystems.ControlPanelSubsystem;

/**
 * A command that will turn the robot to the specified angle.
 */
public class RotateThreeTimes extends PIDCommand {
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetPosition The angle to turn to
   * @param controlPanel   The controlPanel subsystem to use
   */
  public RotateThreeTimes(double targetPosition, ControlPanelSubsystem controlPanel) {
    super(
        new PIDController(PIDConstants.kPanelP, PIDConstants.kPanelI, PIDConstants.kPanelD),
        // Close loop on heading
        controlPanel::getMotorPosition,
        // Set reference to target
        targetPosition,
        // Pipe output to turn robot
        output -> controlPanel.setMotorPower(output),
        // Require the drive
        controlPanel);

  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}