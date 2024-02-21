// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.math.MathUtil;

/** Add your docs here. */
public final class k {
    public static class MATH {
        public static final double DEGREES_TO_RADIANS = 0.017453292519943295;
        public static final double RADIANS_TO_DEGREES = 57.29577951308232;
    }
    public static class RIO_CAN_BUS_IDS {
       public static int AMP_ROTATE_MOTOR = 45;
       public static int AMP_LEFT_SPIN_MOTOR = 46;
       public static int AMP_RIGHT_SPIN_MOTOR = 47;
    }
    public static class OI {
        public static int DRIVER_CONTROLLER_PORT = 0;
        public static int OPERATOR_CONTROLLER_PORT = 1;
    }
    public static class AMP {
        public static final double ROTATE_MOTOR_GEAR_RATIO = 5*4*4;
        public static final double ROTATE_MOTOR_MAX_SPEED_RPM = 5676;
        public static final double ROTATE_MOTOR_MAX_SPEED_RPS = ROTATE_MOTOR_MAX_SPEED_RPM / 60.0;
        public static final double ROTATE_MOTOR_MAX_SPEED_DEG_PER_SEC = ROTATE_MOTOR_MAX_SPEED_RPS * 360;
        public static final double ROTATE_MOTOR_MAX_SPEED_RAD_PER_SEC = ROTATE_MOTOR_MAX_SPEED_DEG_PER_SEC * k.MATH.DEGREES_TO_RADIANS;
        public static final double ROTATE_MOTOR_VELOCITY_SCALE = .0001;
        public static final double ROTATE_MOTOR_FF_KV = k.ROBOT.MAX_BATTERY_VOLTAGE * 1.0 / ROTATE_MOTOR_MAX_SPEED_RAD_PER_SEC ;
        public static final double ROTATE_MOTOR_FF_KA = (ROTATE_MOTOR_FF_KV) * 10.0;
    }
    public static class ROBOT {
        public static final double MAX_BATTERY_VOLTAGE = 12.5;
    }
}
