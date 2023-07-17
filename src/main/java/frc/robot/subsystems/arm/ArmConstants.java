package frc.robot.subsystems.arm;

import com.ma5951.utils.RobotConstants;

public class ArmConstants {
    public static int SecondLevelArmMotorID = 3;

    public static double SecondLevelArmMotorTolorance = Math.toRadians(1);

    public static double SecondLevelArmMotorMaxRotation = Math.toRadians(228.7793036082198);
    public static double SecondLevelArmMotorMinRotation = Math.toRadians(-45.615233956822564);

    public static double SecondLevelArmKP = 1.1;
    public static double SecondLevelArmKI = 0;
    public static double SecondLevelArmKD = 0;

    public static double SecondLevelArmPositionConversionFactor = (2 * Math.PI);

    public static int FirstLevelArmMotorID = 4;

    public static double FirstLevelArmKP = 0.13;
    public static double FirstLevelArmKI = 0;
    public static double FirstLevelArmKD = 0;

    public static double FirstLevelArmPositionConversionFactor = 
        2 * Math.PI / (2048 * 144);

    public static double FirstLevelArmDisFromMassCenter = 0.2035;
    public static double SecondLevelArmDisFromMassCenter = 0.05;

    public static double FirstLevelArmMass = 0.203;
    public static double SecondLevelArmMass = 0.05;

    public static double FirstLevelArmMaxRPM = 5676 / 144d;
    public static double FirstLevelArmKV = FirstLevelArmMaxRPM / 
        RobotConstants.MAX_VOLTAGE;
    public static double FirstLevelArmKT = 
        60 / (2 * Math.PI * FirstLevelArmKV);

    public static double FirstLevelArmTolorance = Math.toRadians(1);

    public static double FirstLevelArmMaxRotation = Math.toRadians(182);
    public static double FirstLevelArmMinRotation = Math.toRadians(-4);

    public static double FirstLevelArmLength = 0.407;
    public static double SecondLevelArmLength = 0.1;

    public static int hallEffectChanel = 0; // TODO
}