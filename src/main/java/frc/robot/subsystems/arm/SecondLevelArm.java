// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ma5951.utils.MAShuffleboard;
import com.ma5951.utils.MAShuffleboard.pidControllerGainSupplier;
import com.ma5951.utils.subsystem.DefaultControlSubsystemInSubsystemControl;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SecondLevelArm extends SubsystemBase implements 
  DefaultControlSubsystemInSubsystemControl{
  /** Creates a new SecondLevelArm. */
  private CANSparkMax motor;
  private SparkMaxPIDController pidController;
  private RelativeEncoder encoder;
  private double setPoint = 0.5 * Math.PI;
  private MAShuffleboard board;
  private pidControllerGainSupplier pidSupplier;

  private static SecondLevelArm instance;

  private SecondLevelArm() {
    motor = new CANSparkMax(
      ArmConstants.SecondLevelArmMotorID, MotorType.kBrushless);
        
    pidController = motor.getPIDController();

    board = new MAShuffleboard("SecondLevelArm");

    pidSupplier = board.getPidControllerGainSupplier(
      ArmConstants.FirstLevelArmKP,
      ArmConstants.FirstLevelArmKI,
      ArmConstants.FirstLevelArmKD
    );

    encoder = motor.getAlternateEncoder(
      SparkMaxAlternateEncoder.Type.kQuadrature, 4096);

    encoder.setInverted(false);
    motor.setInverted(false);
    
    encoder.setPositionConversionFactor(
      ArmConstants.SecondLevelArmPositionConversionFactor);
    
    pidController.setFeedbackDevice(encoder);

    motor.setIdleMode(IdleMode.kCoast);
    motor.setInverted(true);

    
    encoder.setPosition(0.5 * Math.PI);

    pidController.setP(ArmConstants.SecondLevelArmKP);
    pidController.setI(ArmConstants.SecondLevelArmKI);
    pidController.setD(ArmConstants.SecondLevelArmKD);

  }

  public double getRotation() {
    return encoder.getPosition();
  }

  public double getRelativeRotation() {
    return getRotation() - FirsLevelArm.getInstance().getRotation();
  }

  @Override
  public void calculate(double setPoint) {
    pidController.setReference(
      setPoint, ControlType.kPosition);
  }

  @Override
  public boolean atPoint() {
    return Math.abs(getRotation() - setPoint) <=
     ArmConstants.SecondLevelArmMotorTolorance;
  }

  @Override
  public void setVoltage(double voltage) {
    motor.set(voltage / 12);
  }

  @Override
  public boolean canMove() {
    return setPoint < ArmConstants.SecondLevelArmMotorMaxRotation &&
      setPoint > ArmConstants.SecondLevelArmMotorMinRotation;
  }

  @Override
  public void setSetPoint(double setPoint) {
    this.setPoint = setPoint;
  }

  @Override
  public double getSetPoint() {
    return setPoint;
  }

  public static SecondLevelArm getInstance() {
    if (instance == null) {
      instance = new SecondLevelArm();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    board.addNum("Rotation", Math.toDegrees(getRotation()));

    pidController.setP(pidSupplier.getKP());
    pidController.setI(pidSupplier.getKI());
    pidController.setD(pidSupplier.getKD());

  }
}
