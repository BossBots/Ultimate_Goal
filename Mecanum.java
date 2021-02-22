package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


/*
Mecanum.java is a class that handles complete mecanum functionalities.
motor1: front left motor
motor2: front right motor
motor3: back right motor
motor4: back left motor
imu: built-in BNO055IMU in Rev Expansion Hub
*/
public class Mecanum {
    // IMU variables
    private BNO055IMU imu;
    private com.qualcomm.hardware.bosch.BNO055IMU.Parameters imuParameters;

    // motion variables
    private long startTime;
    private double currentAngle;

    // DcMotor objects
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;

    // power values for motors
    private double p1;
    private double p2;
    private double p3;
    private double p4;

    // constructor
    public Mecanum(BNO055IMU i, DcMotor m1, DcMotor m2, DcMotor m3, DcMotor m4) {
        imu = i;
        motor1 = m1;
        motor2 = m2;
        motor3 = m3;
        motor4 = m4;

        // Create new IMU Parameters object.
        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Express acceleration as m/s^2.
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        imu.initialize(imuParameters);
    }

    // constant speed
    public void constantSpeed() {
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // constant power
    public void constantPower() {
      motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // helper function to return z-axis orientation as measured by IMU
    private double getHeading() {
      return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    // forward function for specified power, angle, and interval (reverse is negative power)
    public void forward(double power, double angle, long interval) {
        startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < interval) {
          currentAngle = getHeading();
          if (currentAngle - angle > 5) {
            drive(power, power/2, 0);
        } else if (currentAngle-angle < -5) {
            drive(power, -power/2, 0);
          } else {
            drive(power, 0, 0);
          }
        }
        startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < 250) {
            drive(-power/4,0,0);
        }
        brake(250);
    }

    // yaw function to rotate the frame for specified power and target angle (cw is + power, ccw is - power)
    public void yaw(double power, double angle) {
        currentAngle = getHeading();
        if (angle != 180){
          while (Math.abs(currentAngle - angle) > 3) {
            currentAngle = getHeading();
            drive(0, power, 0);
          }
        } else {
          while (Math.abs(angle - Math.abs(currentAngle)) > 3) {
            currentAngle = getHeading();
            drive(0, power, 0);
          }
        }
        brake(250);
    }

    // drift function for specified power and duration (right is + power, left is - power)
    public void drift(double power, long dur) {
        startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < dur) {
            drive(0, 0, power);
        }
        startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < 250) {
            drive(0, 0, -power);
        }
        brake(250);
    }

    // brake function
    public void brake(long dur) {
        drive(0, 0, 0);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < dur) {
            continue;
        }
    }

    // master drive function for specified forward, yaw, and drift power values
    public void drive(double forward, double yaw, double drift) {
        if (Math.abs(yaw) < 0.15) {
            yaw = 0;
        }
        if (Math.abs(drift) < 0.15) {
            drift = 0;
        }

        p1 = forward + yaw + drift;
        p2 = forward - yaw - drift;
        p3 = forward - yaw + drift;
        p4 = forward + yaw - drift;

        if (p1 > 1) {
            p1 = 1;
        }
        else if (p1 < -1) {
            p1 = -1;
        }
        if (p2 > 1) {
            p2 = 1;
        }
        else if (p2 < -1) {
            p2 = -1;
        }
        if (p3 > 1) {
            p3 = 1;
        }
        else if (p3 < -1) {
            p3 = -1;
        }
        if (p4 > 1) {
            p4 = 1;
        }
        else if (p4 < -1) {
            p4 = -1;
        }
        motor1.setPower(p1);
        motor2.setPower(-p2);
        motor3.setPower(-p3);
        motor4.setPower(p4);
    }
}
