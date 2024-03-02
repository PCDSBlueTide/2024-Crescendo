// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import java.util.ResourceBundle.Control;

import javax.swing.text.html.parser.Element;

import com.ctre.phoenix6.hardware.TalonFX;

import java.util.ArrayList;

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
  private TalonFX intakeMotor = new TalonFX(5);
  private TalonFX rightOuttakeMotor = new TalonFX(6);
  private TalonFX leftOuttakeMotor = new TalonFX(7);
  private ArrayList<CANSparkMax> motors  = new ArrayList(4);

  private DifferentialDrive diffDrive = new DifferentialDrive(motorFrontLeft, motorFrontRight);

  private PS5Controller controller = new PS5Controller(0);

  private boolean enabled = true;
  private boolean inputEnabled = false;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    motors.add(motorFrontLeft);
    motors.add(motorFrontRight);
    motors.add(motorRearLeft);
    motors.add(motorRearRight);
    motorRearLeft.follow(motorFrontLeft);
    motorRearRight.follow(motorFrontRight);
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
    if (controller.getCrossButtonPressed()) {
      enabled = !enabled;
    }
    
    if (!enabled) {
      intakeMotor.stopMotor();
      return;
    }

    if (controller.getR2Button()) {
      intakeMotor.set(-0.3);
    } else if (controller.getR1Button()) {
      //Eject
      intakeMotor.set(0.3);
    } else {
      intakeMotor.set(0);
    }

    if (controller.getL2Button()) {
      rightOuttakeMotor.set(0.45);
      leftOuttakeMotor.set(-0.45);
    } else if (controller.getL1Button()) {
      rightOuttakeMotor.set(0.1);
      leftOuttakeMotor.set(-0.1);
    } else {
      rightOuttakeMotor.set(0);
      leftOuttakeMotor.set(0);
    }

    diffDrive.arcadeDrive(-controller.getRightX(), -controller.getRightY());
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