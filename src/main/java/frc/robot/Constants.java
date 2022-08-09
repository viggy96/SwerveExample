// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.utils.TalonPIDConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double ROBOT_LOOP_PERIOD = 1.0 / 60.0;
    public static final int BLINKIN_LED_CONTROLLER_PORT = 0;

    public static final String RIO_CAN_BUS = "rio";
    public static final String CANIVORE_CAN_BUS = "canivore";

    public static final double DRIVE_GEAR_RATIO = 1.0;
    public static final double DRIVE_WHEEL_DIAMETER_METERS = 0.1524;

    public static final int LF_DRIVE_MOTOR_PORT = 0;
    public static final int LF_ROTATE_MOTOR_PORT = 1;
    public static final int LF_ROTATE_ENCODER_PORT = 2;
    public static final int RF_DRIVE_MOTOR_PORT = 3;
    public static final int RF_ROTATE_MOTOR_PORT = 4;
    public static final int RF_ROTATE_ENCODER_PORT = 5;
    public static final int LR_DRIVE_MOTOR_PORT = 6;
    public static final int LR_ROTATE_MOTOR_PORT = 7;
    public static final int LR_ROTATE_ENCODER_PORT = 8;
    public static final int RR_DRIVE_MOTOR_PORT = 9;
    public static final int RR_ROTATE_MOTOR_PORT = 10;
    public static final int RR_ROTATE_ENCODER_PORT = 11;

    public static final TalonPIDConfig ROTATE_MOTOR_CONFIG = new TalonPIDConfig(false, 
                                                                                false, 
                                                                                4096,
                                                                                6380,
                                                                                0.1,
                                                                                0.0,
                                                                                0.0,
                                                                                0.95,
                                                                                0.1,
                                                                                0,
                                                                                360,
                                                                                false,
                                                                                6380,
                                                                                12760,
                                                                                1);
}
