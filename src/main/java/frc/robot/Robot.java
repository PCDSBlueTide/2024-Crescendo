// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import java.util.ResourceBundle.Control;

import javax.swing.text.html.parser.Element;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;

import java.sql.Driver;
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
  private CANSparkMax liftMotorA = new CANSparkMax(9, CANSparkLowLevel.MotorType.kBrushed);
  private CANSparkMax liftMotorB = new CANSparkMax(10, CANSparkLowLevel.MotorType.kBrushed);
  private TalonFX intakeMotor = new TalonFX(5);
  private TalonFX rightOuttakeMotor = new TalonFX(6);
  private TalonFX leftOuttakeMotor = new TalonFX(7);
  private ArrayList<CANSparkMax> motors  = new ArrayList(4);

  private DifferentialDrive diffDrive = new DifferentialDrive(motorFrontLeft, motorFrontRight);

  private PS5Controller controller = new PS5Controller(0);

  private boolean enabled = true;
  private boolean hasNote = false;
  

  private VelocityVoltage velVoltIntake = new VelocityVoltage(0);
  private VelocityVoltage velVoltRightOutake = new VelocityVoltage(0);
  private VelocityVoltage velVoltLeftOutake = new VelocityVoltage(0);

  private AddressableLED ledStrip = new AddressableLED(0);
  private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(30);

  private Timer autoTimer = null;
  private final Timer moveTimer = new Timer();
  private final Timer unmoveTimer = new Timer();
  private final Timer shootTimer = new Timer();

  private final SendableChooser<Double> speakerSpeed = new SendableChooser<>();

  private DigitalInput noteLimitSwitch = new DigitalInput(0);
  //rps
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
    liftMotorB.follow(liftMotorA);
    var slot0Configs = new Slot0Configs();
    slot0Configs.kV = (double) 1 /40;
    slot0Configs.kP = (double) 1 /40;
    slot0Configs.kI = 0.48;
    slot0Configs.kD = 0.01;
    var slot1Configs = new Slot1Configs();
    slot1Configs.kV = (double) 1 /30;
    slot1Configs.kP = (double) 1 /30;
    slot1Configs.kI = 0.48;
    slot1Configs.kD = 0.01;
    var slot2Configs = new Slot2Configs();
    slot2Configs.kV = (double) 1 /9;
    slot2Configs.kP = (double) 1 /9;
    slot2Configs.kI = 0.48;
    slot2Configs.kD = 0.01;
    intakeMotor.getConfigurator().apply(slot0Configs, 0.050);
    rightOuttakeMotor.getConfigurator().apply(slot1Configs, 0.050);
    leftOuttakeMotor.getConfigurator().apply(slot1Configs, 0.050);
    rightOuttakeMotor.getConfigurator().apply(slot2Configs, 0.050);
    leftOuttakeMotor.getConfigurator().apply(slot2Configs, 0.050);
    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();
    speakerSpeed.addOption("Slow", 30.0);
    speakerSpeed.setDefaultOption("Normal", 40.0);
    speakerSpeed.addOption("+5", 45.0);
    speakerSpeed.addOption("+10", 50.0);
    SmartDashboard.putData(speakerSpeed);

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
    SmartDashboard.putBoolean("noteIn", noteLimitSwitch.get());
    if (controller.getCrossButtonPressed()) {
      enabled = !enabled;
    }

    if (!enabled) {
      setAllLights(255, 0, 0);
      motors.forEach((motor) -> motor.set(0));
      liftMotorA.set(0);
      intakeMotor.set(0);
      rightOuttakeMotor.set(0);
      leftOuttakeMotor.set(0);
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
        setAllLights(255, 0, 0);
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autoTimer = new Timer();
    autoTimer.start();
    shootTimer.stop();
    unmoveTimer.stop();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    if (!enabled) return;

    double time = autoTimer.get();

    if (time <= 1.0) {
      shootSpeaker();
      setAllLights(255, 115, 0);
    } else if (time <= 2.0) {
      shootSpeaker();
      intake();
      setAllLights(0, 255, 221);
    } else if (time <= 7.0 && !noteLimitSwitch.get()) {
      moveTimer.start();
      intake();
      stopOutake();
      diffDrive.arcadeDrive(-0.15, 0);
    } 
    else {
      moveTimer.stop();
      //Shouldnt be on
      hasNote = true;
    }
    if(hasNote) {
      unmoveTimer.start();
      if(unmoveTimer.get() <= moveTimer.get()) {
        diffDrive.arcadeDrive(0.15, 0);
      }
      else {
        hasNote = false;
      }
    }
    else if(unmoveTimer.get() >= moveTimer.get()) {
      shootTimer.start();
      if(shootTimer.get() <= 1.0) {
        shootSpeaker();
        setAllLights(255, 115, 0);
      }
      else if(shootTimer.get() <= 2.0)  {
        shootSpeaker();
        intake();
        setAllLights(0, 255, 221);
      }
    }
  }

  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    setAllLights(0, 255, 0);
    if (controller.getCircleButton()) {
      liftMotorA.set(0.6);
    }
    else if (controller.getSquareButton()) {
      liftMotorA.set(-0.6);
    }
    else {
      liftMotorA.set(0);
    }

    if (controller.getR2Button()) {
        intake();
        setAllLights(50, 200, 255);
    } else if (controller.getR1Button()) {
      //Eject 
      intakeMotor.set(0.3);
    } else {
      intakeMotor.set(0);
    }
    if (controller.getL2Button()) {
      shootSpeaker();
    }
    else if (controller.getL1Button()) {
      shootAmp();
    }
    else if (controller.getPSButton()) {
      reverseShooter();
    }
    else {
      rightOuttakeMotor.set(0);
      leftOuttakeMotor.set(0);
    }

    diffDrive.arcadeDrive(-controller.getRightX(), -controller.getRightY());
    DriverStation.reportWarning(""+noteLimitSwitch.get(), false);
  }

    /**
     * Runs the intake
      */
  private void intake() {
      //intakeMotor.set(-0.3);
      velVoltIntake.Slot = 0;
      intakeMotor.setControl(velVoltIntake.withVelocity(-40));
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

  /**
   * Sets all the lights to one color
   * @param r red
   * @param g green
   * @param b blue
   */
  private void setAllLights(int r, int g, int b) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, r, g, b);
    }
    ledStrip.setData(ledBuffer);
  }

  /**
   * Shoots on the speaker with 30 velocity
   */
  private void shootSpeaker() {
    double speed = speakerSpeed.getSelected();
    velVoltRightOutake.Slot = 1;
    rightOuttakeMotor.setControl(velVoltRightOutake.withVelocity(speed));
    velVoltLeftOutake.Slot = 1;
    leftOuttakeMotor.setControl(velVoltLeftOutake.withVelocity(-speed));
    SmartDashboard.putString("outakeSpeed", "Leftoutake Speed: " + leftOuttakeMotor.getVelocity() + " Rightoutake Speed: " + rightOuttakeMotor.getVelocity());
    setAllLights(0, 0, 255);
  }

  /**
   * Shoots on the amp with 9 velocity
   */
  private void shootAmp() {
    double speed = 9.0;
    velVoltRightOutake.Slot = 2;
    rightOuttakeMotor.setControl(velVoltRightOutake.withVelocity(speed));
    velVoltLeftOutake.Slot = 2;
    leftOuttakeMotor.setControl(velVoltLeftOutake.withVelocity(-speed));
    SmartDashboard.putString("outakeSpeed", "Leftoutake Speed: " + leftOuttakeMotor.getVelocity() + " Rightoutake Speed: " + rightOuttakeMotor.getVelocity());
    setAllLights(255, 0, 255);
  }

  private void stopOutake() {
    rightOuttakeMotor.set(0);
    leftOuttakeMotor.set(0);
  }
  private void reverseShooter() {
    rightOuttakeMotor.set(-0.1);
    leftOuttakeMotor.set(-0.1);
  }
}