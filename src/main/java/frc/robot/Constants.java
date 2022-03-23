// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Buttons
    public static final int gyroButton = 9;

    //Joystick
    public static final int forwardAxis = 1;
    public static final int strafeAxis = 0;
    public static final int rotationAxis = 2;

    public static final double driveSpeedCap = 0.8; //Percent of max speed
    public static final double rotationSpeedCap = 1; 
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5715; // FIXME Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5715; // FIXME Measure and set wheelbase

    // public static final int DRIVETRAIN_PIGEON_ID = 0; // FIXME Set Pigeon ID

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 2; // FIXME Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 4; // FIXME Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 3; // FIXME Set front left steer encoder ID
    // public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(271.230469); // 360-88.769531
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(265.251160); // 360-88.769531

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 5; // FIXME Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7; // FIXME Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 6; // FIXME Set front right steer encoder ID
    // public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(94.482422); // 360-85.517578+180
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(83.317566);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 11; // FIXME Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 13; // FIXME Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 12; // FIXME Set back left steer encoder ID
    // public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(164.970703); // 360-15.0292971+180
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(16.960144);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 8; // FIXME Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 10; // FIXME Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 9; // FIXME Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(156.349182);//139.471); // 360-155.654297+180

    // Robot
    // units are in inches!!
    public static final double trackWidth = 22.5;
    public static final double wheelBase = 22.5;

    public static final int PH_ID = 31;
    public static final int HOOK_SOLENOID_1 = 1;
    public static final int HOOK_SOLENOID_2 = 2;
    public static final int LIFTER_SOLENOID = 7;

         
    //THESE UNITS ARE IN INCHES!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! (!!)
    public static final double reflectiveStripWidth = 9.0;
    public static final double reflectiveStripHeight = 10.0;

    public static final double LIMELIGHTHEIGHT = 39;
    public static final double HUBHEIGHT = 105;


    // constant thing - no touchie
    public static final double TOINCHES = 0.81442;
    public static final double thing = 1.5;


    // In degrees
    public static final double LIMELIGHTANGLE = 25;
    public static final double TURRET_TOLERANCE = 1;
    public static final double SHOOTER_ENCODER_OFFSET = 0;


    //In rotations; shooter degree of movement clockwise and counterclockwise from the centerline
    public static final double SHOOTER_MAX_ROT_CLOCKWISE = 180.0;//45/8;
    public static final double SHOOTER_MAX_ROT_COUNTERCLOCK = 180;//-45/8;

    //can id
    public static final int TURN_MOTOR_ID = 23;
    public static final int SHOOTER_MOTOR_1_ID = 25;
    public static final int SHOOTER_MOTOR_2_ID = 21;
    public static final int TURRET_ENCODER_ID = 9;
    public static final int SHOOTER_ENCODER_ID = 8;
    //public static final int SHOOTER_ENCODER_ID = 9;
    public static final int INTAKE_MOTOR_ID = 30;
    public static final int STORAGE_MOTOR_ID = 22;
    public static final int FEEDER_MOTOR_ID = 24;
    //public static final int INTAKE_MOTOR_ID = ;
    //public static final int DEPLOY_INTAKE_MOTOR_ID = ;
    //public static final int 

    //PID constants: index 0 represents P, 1 represents I, 2, represents D
    public static final double[] TURN_MOTOR_PID = new double[] {0.02, 0.0002, 10, 0.0};/*
    public static final double[] MASTER_SHOOTER_PID = new double[] {0.0004, 0.000001, 0.01};//.0004};
    public static final double[] SLAVE_SHOOTER_PID = new double[] {0.0004, 0.000001, 0.01};//0004};
*/
    public static final double[] MASTER_SHOOTER_PID = new double[] {0.0025, 0.000000097, 5};//.0004};
    public static final double[] SLAVE_SHOOTER_PID = MASTER_SHOOTER_PID;
    //
    public static final double[] FEEDFORWARD = new double[] {0.00007,0};//{0.00002, 0.00002};

    //le data
    public static final double[] DISTANCE = {77.476902, 95, 131.466488, 157, 185.365014, 204.941936, 242};
    public static final double[] SPEED =    {-2300, -2400, -2600, -2900, -3100, -3250, -3600};
    public static final double TRANSLATE = -1636.1;
    public static final double SCALER = -7.9555;
    public static final double SHOOTER_SPROCKET_RATIO = 180/16.0;

    // les speed
    public static final double FLYWHEEL_TOLERANCE = 12;

    //public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(24.345703); // 360-155.654297+180

    public static final double TURRET_TURN_SPEED = 90;
}

//181.406250
//-1.757813
//170.332031
//-97.382813 