// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.FirsLevelArm;
import frc.robot.subsystems.arm.SecondLevelArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetArm extends InstantCommand {
  private final double x;
  private final double y;

  public SetArm(double x, double y) {
    this.x = x;
    this.y = y;
  }

  private static double optimizeAngle(double angle) {
    if (Math.abs(angle) > Math.PI) {
      double times = Math.abs(Math.floor(angle / Math.PI));
      angle = angle - Math.signum(angle) * Math.PI * times;
      angle = times % 2 == 0 ? angle : angle - Math.PI;
    }
    return angle;
  }

  private static double getDisFromPose(double a, double b) {
    return Math.abs(FirsLevelArm.getInstance().getRotation() - a)
        + Math.abs(SecondLevelArm.getInstance().getRotation() - b);
  }

  private static boolean inRange(double a, double b) {
    return a < ArmConstants.FirstLevelArmMaxRotation
      && a > ArmConstants.FirstLevelArmMinRotation
      && b < ArmConstants.SecondLevelArmMotorMaxRotation
      && b > ArmConstants.SecondLevelArmMotorMinRotation;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // q - second level arm relative angle
    // a - first level srm angle
    // b - second level arm absolute angle
    double cosQ = (x * x + y * y - Math.pow(ArmConstants.FirstLevelArmLength, 2)
        - Math.pow(ArmConstants.SecondLevelArmLength, 2)) /
        2 * ArmConstants.SecondLevelArmLength * ArmConstants.FirstLevelArmLength;

    if (Math.abs(cosQ) > 1) {
      return;
    }

    double q1 = Math.acos(cosQ), q2 = -Math.acos(cosQ);

    if (x < 0) {
      q1 += Math.PI;
      q2 += Math.PI;
    }

    double a1 = Math.atan(y / x) -
        Math.atan((ArmConstants.SecondLevelArmLength * Math.sin(q1)) /
            ArmConstants.FirstLevelArmLength
            + ArmConstants.SecondLevelArmLength
                * Math.cos(q1));

    double a2 = Math.atan(y / x) -
        Math.atan((ArmConstants.SecondLevelArmLength * Math.sin(q2)) /
            ArmConstants.FirstLevelArmLength
            + ArmConstants.SecondLevelArmLength
                * Math.cos(q2));

    if (x < 0) {
      a1 += Math.PI;
      a2 += Math.PI;
    }

    double b1 = q1 + a1;
    double b2 = q2 + a2;

    a1 = optimizeAngle(a1);
    b1 = optimizeAngle(b1);
    a2 = optimizeAngle(a2);
    b2 = optimizeAngle(b2);

    if (getDisFromPose(a2, b2) < getDisFromPose(a1, b1) && inRange(a2, b2)) {
      FirsLevelArm.getInstance().setSetPoint(a2);
      SecondLevelArm.getInstance().setSetPoint(b2);
    } else {
      FirsLevelArm.getInstance().setSetPoint(a1);
      SecondLevelArm.getInstance().setSetPoint(b1);
    }
  }
}
