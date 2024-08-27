// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.preferences.PrefBool;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public HashMap<Integer, TalonFX> motors = new HashMap<Integer, TalonFX>();
  public HashMap<Integer, VelocityVoltage> velocityRequests = new HashMap<Integer, VelocityVoltage>();
  public HashMap<Integer, PrefBool> enabled = new HashMap<Integer, PrefBool>();
  public ShuffleboardTab tab = Shuffleboard.getTab("MotorTesting");
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Constants.kRPM.loadPreferences();
    Constants.kP.loadPreferences();
    Constants.kI.loadPreferences();
    Constants.kD.loadPreferences();
    Constants.kV.loadPreferences();
    for (Constants.MotorIDs id : Constants.MotorIDs.values()) {
      int i = id.id;
      TalonFX motor = new TalonFX(i);
      VelocityVoltage velocityRequest = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
      motor.setControl(velocityRequest);
      motors.put(i, motor);
      velocityRequests.put(i, velocityRequest);
      PrefBool b = new PrefBool("Motor" + Integer.toString(i), false);
      enabled.put(i, b);
      b.loadPreferences();
      tab.addDouble("Motor" + Integer.toString(i) + " Velocity", () -> motor.getVelocity().getValueAsDouble());
    }
    SmartDashboard.putData("Stop All", stopAllMotorsCommand());
    SmartDashboard.putData("Set RPM of Enabled", setRPMsOfEnabledCommand());
  }

  void stopAllMotors() {
    for (Constants.MotorIDs id : Constants.MotorIDs.values()) {
      velocityRequests.get(id.id).Velocity = 0;
      motors.get(id.id).setVoltage(0);
    }
  }

  void setRPMsOfEnabled() {
    enabled.get(0).loadPreferences();
    Constants.kRPM.loadPreferences();
    Constants.kP.loadPreferences();
    Constants.kI.loadPreferences();
    Constants.kD.loadPreferences();
    Constants.kV.loadPreferences();
    for (Constants.MotorIDs id : Constants.MotorIDs.values()) {
      enabled.get(id.id).loadPreferences();
      if (enabled.get(id.id).get()) {
        velocityRequests.get(id.id).Velocity = Constants.kRPM.get();
        motors.get(id.id).setControl(velocityRequests.get(id.id));
        if (Math.abs(Constants.kRPM.get()) < 0.01) {
          motors.get(id.id).setVoltage(0);
        }
        setMotorPID(motors.get(id.id), Constants.kP.get(), Constants.kI.get(), Constants.kD.get(), Constants.kV.get());
      }
    }
  }

  Command stopAllMotorsCommand() {
    return Commands.runOnce(() -> stopAllMotors());
  }

  Command setRPMsOfEnabledCommand() {
    return Commands.runOnce(() -> setRPMsOfEnabled());
  }

  void setMotorPID(TalonFX mootor, double perbosity, double ierbosity, double derbosity, double verbosity) {
    TalonFXConfigurator configurator = mootor.getConfigurator();
    TalonFXConfiguration config = new TalonFXConfiguration();
    configurator.refresh(config);
    config.Slot0.kP = perbosity;
    config.Slot0.kI = ierbosity;
    config.Slot0.kD = derbosity;
    config.Slot0.kV = verbosity;
    StatusCode status = configurator.apply(config);
    if (!status.isOK()){
      DriverStation.reportError("Could not apply shooter configs, error code:"+ status.toString(), new Error().getStackTrace());
    }
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
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    

    CommandScheduler.getInstance().run();
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
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
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
