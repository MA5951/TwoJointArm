// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ma5951.utils.MAShuffleboard;
import com.ma5951.utils.RobotConstants;
import com.ma5951.utils.MAShuffleboard.pidControllerGainSupplier;
import com.ma5951.utils.subsystem.DefaultControlSubsystemInSubsystemControl;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FirsLevelArm extends SubsystemBase implements 
  DefaultControlSubsystemInSubsystemControl{
  /** Creates a new firsLevelArm. */
  private TalonFX motor;
  private MAShuffleboard board;
  private double setPoint = Math.PI * 0.5;
  private pidControllerGainSupplier pidSupplier;
  // private DigitalInput hallEffect;

  private static FirsLevelArm instance;

  private FirsLevelArm() {
    motor = new TalonFX(ArmConstants.FirstLevelArmMotorID);
    motor.setInverted(true);
    board = new MAShuffleboard("FirsLevelArm");
    pidSupplier = board.getPidControllerGainSupplier(
      ArmConstants.FirstLevelArmKP,
      ArmConstants.FirstLevelArmKI,
      ArmConstants.FirstLevelArmKD
    );
    motor.configClosedloopRamp(0);
    motor.config_kP(0, ArmConstants.FirstLevelArmKP);
    motor.config_kI(0, ArmConstants.FirstLevelArmKI);
    motor.config_kD(0, ArmConstants.FirstLevelArmKD);

    // hallEffect = new DigitalInput(ArmConstants.hallEffectChanel);
    motor.setSelectedSensorPosition((0.5 * Math.PI) / 
      ArmConstants.FirstLevelArmPositionConversionFactor);
    motor.setNeutralMode(NeutralMode.Coast);



  }

  public double getRotation() {
    return 
      (motor.getSelectedSensorPosition() * 
      ArmConstants.FirstLevelArmPositionConversionFactor);
  }

  public double getFeed() {
    double d1x = 
      ArmConstants.FirstLevelArmDisFromMassCenter * Math.cos(getRotation());
    double d1y =  
      ArmConstants.FirstLevelArmDisFromMassCenter * Math.sin(getRotation());
    double d2x = 
      d1x + ArmConstants.SecondLevelArmDisFromMassCenter * 
      Math.cos(SecondLevelArm.getInstance().getRotation());
    double d2y = 
      d1x + ArmConstants.SecondLevelArmDisFromMassCenter * 
      Math.sin(SecondLevelArm.getInstance().getRotation());
    double centerOfMassAngle = Math.atan(
      (d2y * ArmConstants.SecondLevelArmMass
      + d1y * ArmConstants.FirstLevelArmMass)
      / (d2x * ArmConstants.SecondLevelArmMass
      + d1x * ArmConstants.FirstLevelArmMass)
    );
    double centerOfMassMoment = (d2x * ArmConstants.SecondLevelArmMass +
                              d1x * ArmConstants.FirstLevelArmMass)
                              / (Math.cos(centerOfMassAngle));
    return ((centerOfMassMoment * Math.cos(centerOfMassAngle)
      * RobotConstants.KGRAVITY_ACCELERATION) / ArmConstants.FirstLevelArmKT)
      / RobotConstants.MAX_VOLTAGE;
  }

  @Override
  public void calculate(double setPoint) {
    motor.set(ControlMode.Position, setPoint /
      ArmConstants.FirstLevelArmPositionConversionFactor,
      DemandType.ArbitraryFeedForward, getFeed());    
  }

  @Override
  public boolean atPoint() {
    return Math.abs(setPoint - getRotation()) <=
     ArmConstants.FirstLevelArmTolorance;
  }

  @Override
  public void setVoltage(double voltage) {
    motor.set(ControlMode.PercentOutput, voltage / 12);    
  }

  @Override
  public boolean canMove() {
    return setPoint < ArmConstants.FirstLevelArmMaxRotation &&
      setPoint > ArmConstants.FirstLevelArmMinRotation;
  }

  @Override
  public void setSetPoint(double setPoint) {
    this.setPoint = setPoint;
  }

  @Override
  public double getSetPoint() {
    return setPoint;
  }

  public static FirsLevelArm getInstance() {
      if (instance == null) {
        instance = new FirsLevelArm();
      }
      return instance;
  }

  @Override
  public void periodic() {

    board.addNum("rotation", Math.toDegrees(getRotation()));

    motor.config_kP(0, pidSupplier.getKP());
    motor.config_kI(0, pidSupplier.getKP());
    motor.config_kD(0, pidSupplier.getKP());
    // if (!hallEffect.get()) {
    //   motor.setSelectedSensorPosition(ArmConstants.FirstLevelArmMinRotation);
    // }
  }
}
