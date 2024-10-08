// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.util.preferences.PrefBool;
import frc.robot.util.preferences.PrefDouble;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static PrefDouble kPosition = new PrefDouble("Position", 0.0);
  public static PrefDouble kRPM = new PrefDouble("RPM", 0.0);
  public static PrefDouble kP = new PrefDouble("P", 0);
  public static PrefDouble kI = new PrefDouble("I", 0);
  public static PrefDouble kD = new PrefDouble("D", 0);
  public static PrefDouble kV = new PrefDouble("V", 0);
  public enum MotorIDs {
    Motor1(0),
    ;

    int id;
    MotorIDs(int _id) {
      id = _id;
    }
  }
}
