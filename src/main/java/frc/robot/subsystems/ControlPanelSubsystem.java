package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.types.ControlPanelColor;
import frc.robot.utilities.ColorMatcher;

/**
 * ControlPanelSubsystem handles the control panel manipulator and sensor.
 */
public class ControlPanelSubsystem extends SubsystemBase {
  private final ColorSensorV3 m_colorSensor;
  private final ColorMatcher m_colorMatcher;
  private ControlPanelColor m_matchedColor;

  private final CANSparkMax m_motor;
  private final CANEncoder m_encoder;
  private CANPIDController m_pidController;

  public ControlPanelSubsystem() {
    // Color sensor
    m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    m_colorMatcher = new ColorMatcher();

    // Configure motor
    m_motor = new CANSparkMax(RobotMap.kControlPanelSparkMax, MotorType.kBrushless);
    m_encoder = m_motor.getEncoder();
    m_pidController = m_motor.getPIDController();

    m_motor.restoreFactoryDefaults();
    m_motor.setIdleMode(IdleMode.kCoast);

    // set PID coefficients
    m_pidController.setP(PIDConstants.kPanelP);
    m_pidController.setI(PIDConstants.kPanelI);
    m_pidController.setD(PIDConstants.kPanelD);
    m_pidController.setIZone(PIDConstants.kPanelIzone);
    m_pidController.setFF(PIDConstants.kPanelFF);
    m_pidController.setOutputRange(PIDConstants.kMinOutput, PIDConstants.kMaxOutput);
  }

  @Override
  public void periodic() {
    Color detectedColor = m_colorSensor.getColor();
    m_matchedColor = m_colorMatcher.getMatchedColor(detectedColor);
    SmartDashboard.putString("Matched Color", m_matchedColor.name());
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
  }

  // Returns an enum of the matched color from 0-4
  public ControlPanelColor getMatchedColor() {
    SmartDashboard.putString("Matched Color ", m_matchedColor.name());
    return m_matchedColor;
  }

  // Get the target color from the game field
  public ControlPanelColor getTargetColor() {
    String gameData;
    ControlPanelColor targetColor = ControlPanelColor.UNKNOWN;
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if(gameData.length() > 0)
    {
      switch (gameData.charAt(0))
      {
        case 'B' :
          //Blue case code
          targetColor = ControlPanelColor.BLUE;
          break;
        case 'G' :
          //Green case code
          targetColor = ControlPanelColor.GREEN;
          break;
        case 'R' :
          //Red case code
          targetColor = ControlPanelColor.RED;
          break;
        case 'Y' :
          //Yellow case code
          targetColor = ControlPanelColor.YELLOW;
          break;
        default :
          //This is corrupt data
          break;
      }
    } else {
      //Code for no data received yet
      targetColor = ControlPanelColor.UNKNOWN
      ;
    }
    SmartDashboard.putString("Target Color ", targetColor.name());
    return targetColor;
  }
  
  // Rotate the control panel the number of specified segments
  public void rotateSegments(double segments) {
    // Calculate the number of manipulator wheel rotations
    double rotations = (segments * RobotMap.kColorArcLength) / RobotMap.kManipulatorCircumference;
    runPositionLoop(rotations);
  }

  // Gets the detected and target color and rotates to target color
  public void rotateToColor() {
    // Get the matched and target color. Add 2 segments to move it to position
    double segments = (getMatchedColor().ordinal() - getTargetColor().ordinal()) + 2;
    // Faster to spin in reverse if segments equals 3
    if (segments == 3) {segments = -1;}

    // Rotate the control panel to the computed number of segment
    SmartDashboard.putNumber("Segment rotation ", segments);
    rotateSegments(segments);
  }

  // Returns whether a color is known
  public BooleanSupplier unknownColor() {
    BooleanSupplier sup = () -> false;
    if (m_matchedColor == ControlPanelColor.UNKNOWN) {
      sup = () -> true;
    }
    return sup;
  }

  // Convenience method for running inline command
  public void rotateHalfSegment() {
    rotateSegments(0.5);
  }

  // -- Motor control methods --

  // Set a power level from 0.0 to 1.0
  public void setMotorPower(double power) {
    m_motor.set(power);
  }

  // Read the motor encoder position
  public double getMotorPosition() {
    return m_encoder.getPosition();
  }

  // Closed position loop using number of rotations as the setpoint
  public void runPositionLoop(double rotations) {
    m_pidController.setReference(rotations, ControlType.kPosition);
  }
}