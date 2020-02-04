package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FeederSubsystem.HopperState;
import frc.robot.subsystems.FeederSubsystem.IndexState;

/**
 * Stops the feeder subsystem
 */
public class StartFeeder extends CommandBase {
  // The subsystem the command runs on
  private final FeederSubsystem m_feeder;

  public StartFeeder(FeederSubsystem subsystem) {
    m_feeder = subsystem;
    addRequirements(m_feeder);
  }

  @Override
  public void initialize() {
      m_feeder.setHopperState(HopperState.STOPPED);
      m_feeder.setIndexState(IndexState.FULL);
  }

  @Override
  public void execute() {
    m_feeder.index();
  }
  
  public void end() {
    m_feeder.setHopperState(HopperState.HALTED);
    m_feeder.setIndexState(IndexState.HALTED);
  }
}