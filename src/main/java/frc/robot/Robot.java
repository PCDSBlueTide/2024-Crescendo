// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkMax;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {


  private CANSparkMax motorFrontLeft = new CANSparkMax(1, CANSparkLowLevel.MotorType.kBrushed);
  private CANSparkMax motorFrontRight = new CANSparkMax(2, CANSparkLowLevel.MotorType.kBrushed);
  private CANSparkMax motorRearLeft = new CANSparkMax(4, CANSparkLowLevel.MotorType.kBrushed);
  private CANSparkMax motorRearRight = new CANSparkMax(3, CANSparkLowLevel.MotorType.kBrushed);

  private DifferentialDrive frontDrive = new DifferentialDrive(motorFrontLeft, motorFrontRight);
  private DifferentialDrive rearDrive = new DifferentialDrive(motorRearLeft, motorRearRight);

  private PS5Controller controller = new PS5Controller(0);

  private boolean isDisabled = false;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    if(controller.getCrossButton())
    {
      isDisabled = true;
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if(isDisabled)
    {
      motorFrontLeft.stopMotor();
      motorFrontRight.stopMotor();
      motorRearLeft.stopMotor();
      motorRearRight.stopMotor();
    }
    else
    {
      frontDrive.arcadeDrive(controller.getRightX(), controller.getRightY());
      rearDrive.arcadeDrive(controller.getRightX(), controller.getRightY());
    }
  }

  @Override
  public void testInit() {

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
