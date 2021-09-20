/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //SWERVE constants
    public static final double maxSpeed = 3.0; // 3 meters per second
    public static final double maxAngularSpeed = Math.PI; // 1/2 rotation per second
    public static final double swerveModuleDistance = 0.381;
    public static final double angleEncoderConversionFactor = 19.969;

    public static int frontLeftSteer = 1;
    public static int frontLeftDrive = 2;
    
    public static int frontRightSteer = 3;
    public static int frontRightDrive = 4;

    public static int rearLeftSteer = 5;
    public static int rearLeftDrive = 6;

    public static int rearRightSteer = 7;
    public static int rearRightDrive = 8;
}
