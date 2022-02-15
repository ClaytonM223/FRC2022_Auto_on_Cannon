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
      //USB
      public static final int JOYSTICK_ID = 0;
      //BUTTONS
      public static final int TUBE_1_BUTTON_ID = 3;
      public static final int TUBE_2_BUTTON_ID = 4;
      public static final int TUBE_3_BUTTON_ID = 5;
      public static final int TUBE_4_BUTTON_ID = 6;
      public static final int HORN_BUTTON_ID = 1;
      public static final int E_STOP = 2;
      //AXIS
      public static final int Y_AXIS_ID = 1;
      public static final int X_AXIS_ID = 0; 
      public static final int Z_AXIS_ID = 2;
      public static final int SLIDER_ID = 3;
      //MOTORS
      public static final int RIGHT_FRONT_MOTOR = 2;
      public static final int LEFT_FRONT_MOTOR = 0;
      public static final int LEFT_BACK_MOTOR = 1;
      public static final int RIGHT_BACK_MOTOR = 3;
      //SOLENOIDS
      public static final int TUBE_1_ID = 1;
      public static final int TUBE_2_ID = 2;
      public static final int TUBE_3_ID = 0;
      public static final int TUBE_4_ID = 3;
      public static final int HORN_ID = 4;

      //Auto numbers
      public static final int TICKS_PER_ROTATION = 2048;
    
}
