/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class RobotMap{

    // Control Panel
    public static final int kControlPanelSparkMax = 69; //Placeholder
    public static final double kControlPanelCircumference = 0.81 * Math.PI;
    public static final double kColorArcLength = kControlPanelCircumference / 8;
    public static final double kManipulatorCircumference = 0.1 * Math.PI;
    public static double threeTurns = 26.0; // Rotate 26 segments 
    
    // PID constants
    public static double kPanelP = 0.1;
    public static double kPanelI = 1e-4;
    public static double kPanelD = 1;
    public static double kPanelIzone = 0;
    public static double kPanelFF = 0;
    public static double kMaxOutput = 1;
    public static double kMinOutput = -1;

    // Shooter
    public static final int kFlywheelTalonFX = 0;
    public static final int kHoodTalonSRX = 5;
    public static final double kTargetHeight = 4.0;
    public static final double kTurretHeight = 1.0;
    public static final double kRelativeTargetHeight = kTargetHeight - kTurretHeight;

    // Intake Solenoids
    public static final int kIntakeSoleniodRightOne = 69; //place holder
    public static final int kIntakeSoleniodRightTwo = 69;
    public static final int kIntakeSoleniodLeftOne = 69;
    public static final int kIntakeSoleniodLeftTwo = 69;

    // Intake motor
    public static final int kIntakeWPI_TalonSRX = 6; //placeholder 

    public static final int kHopperVictorSPX = 420; //Placeholder
    public static final int kTowerVictorSPX = 987; //Placerholder
    public static final int kIRSensorBottom = 0;
    public static final int kIRSensorMiddle = 1;
    public static final int kIRSensorTop = 2;
  }
  
  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 0;
    public static final int kLeftMotor2Port = 1;
    public static final int kRightMotor1Port = 2;
    public static final int kRightMotor2Port = 3;

    public static final int[] kLeftEncoderPorts = new int[]{0, 1};
    public static final int[] kRightEncoderPorts = new int[]{2, 3};
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    public static final boolean kGyroReversed = true;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 8.5;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 1;
  }

  public static final class ConversionConstants{
      
    public static final double kFlywheelEncoderTicksPerRotation = 2048;
    public static final double kFlywheelGearRatio = 24.0/14.0;

    public static final double kHoodEncoderTicksPerRotation = 4096;
    public static final double kHoodGearRatio = 60.0/24.0; 
  }

  public static final class PIDConstants{

    //Shooter
    public static final double kFlywheelkP = 0.075;
    public static final double kFlywheelkF = 0.0467;
    public static final double kHoodkP = 2.5;
    public static final double kHoodkD = 15;

  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
}
