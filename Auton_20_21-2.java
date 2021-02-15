/*
Copyright 2018 FIRST Tech Challenge Team 524

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import java.util.List;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@Autonomous
public class Auton_20_21 extends LinearOpMode {
    private BNO055IMU imu;
    
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;
    
    private DcMotor pivot;
    private DcMotor launch;
    private DcMotor ramp;
    
    
    private CRServo intake;
    private CRServo ramp_rings1;
    private CRServo ramp_rings2;
    private Servo lock;
    private Servo angle_adjust;
    
    // Made these all public because there was some problem with their scope not reaching to a method.
    // Every error is because of some quick, thoughtless botch like this one.
    public static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    public static final String LABEL_FIRST_ELEMENT = "Quad";
    public static final String LABEL_SECOND_ELEMENT = "Single";
    public static final String VUFORIA_KEY = "ASXuYar/////AAABmXFF9U0Sqkf4nhGpOxAwL4hjOruKN+pzxoY06iPyjRCJzC2VJLEcTeWGlru0xqBeYD1/4Q/Re8WeD61/uoVn4xHD2U4ZJcxUOgBIJ9tpv181fPxonQEECTo6DAbx6VaADyWN5deZ5fkOVeQD7He5kCgL7DA5VbYDsXCNoqF4Ifnj+pyVUlYZeuiIvpHUDGXfa6E5a3jJk4p6ksFK0BCObGsRtKfFsCKZvjz8shWT0ifp4majfLUu3J8xLNmEM6pLy9bsDWgANt+Ao+zrFDJ1jUGG92oI1nllBYQCW/PC3to0UeGPIeUWpTSFBZFp8GSAf633CgKcxSftde0rgBxMlrB2YIWeM6bPXPwbqSpvS/yT";
    public VuforiaLocalizer vuforia;
    
    public Recognition[] detections;
    public TFObjectDetector tfod;
    

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        
        pivot = hardwareMap.get(DcMotor.class, "pivot");
        launch = hardwareMap.get(DcMotor.class, "launch");
        ramp = hardwareMap.get(DcMotor.class, "ramp");
        
        
        intake = hardwareMap.get(CRServo.class, "intake");
        lock = hardwareMap.get(Servo.class, "lock");
        ramp_rings1 = hardwareMap.get(CRServo.class, "ramp_rings1");
        ramp_rings2 = hardwareMap.get(CRServo.class, "ramp_rings2");
        angle_adjust = hardwareMap.get(Servo.class, "angle_adjust");
        
        // removed sensor code; replace with camera
        int numRings = -1;
        
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ramp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        launch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ramp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        
        
        com.qualcomm.hardware.bosch.BNO055IMU.Parameters imuParameters;
        double angles;
        boolean i = false;
        double start;
        double init_angle;
        double start_time;
    
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
        
        
        initVuforia();
        initTfod();
        
        tfod.activate();
        
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            ramp.setPower(0.1);
            lock.setPosition(0);
            angle_adjust.setPosition(1);
            launch.setPower(0.7);
            sleep(1000);
            
            
            // wobble goal bending
            while (pivot.getCurrentPosition() < 200) {
                pivot.setPower(0.5);
            }
            while (pivot.getCurrentPosition() > 400) {
                pivot.setPower(-0.25);
            }
            pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pivot.setPower(-0.1);
            
            launcher();
            launch.setPower(0);
            drift(-0.5, 500);
            fd(0.5, 0, 0.5);
            
            // Object Detection goes here
            
            numRings = 1;//(int) (countRings()[0] + 0.5);
            
            // collect and launch the randomization rings
            launch.setPower(0.7);
            intake.setPower(1);
            ramp_rings1.setPower(1);
            ramp_rings2.setPower(1);
            fd(0.5, 0, 0.75);
            launcher();
            launch.setPower(0);
            
            
            if (numRings == 4) {
                fd(0.75, 0, 1);
                drift(-0.5, 1000);
                lock.setPosition(1);
                drift(0.5, 1000);
                fd(-0.75, 0, 1);
            } else if (numRings == 2) {
                fd(0.75, 0, 0.75);
                drift(-0.5, 500);
                lock.setPosition(1);
                drift(0.5, 500);
                fd(-0.75, 0, 0.75);
            } else {
                fd(0.75, 0, 0.5);
                drift(-0.5, 1000);
                lock.setPosition(1);
                drift(0.5, 1000);
                fd(-0.75, 0, 0.5);
            }
            
            
            // pick up 2nd wobble goal
            drift(-0.25, 500);
            lock.setPosition(0);
            drift(0.25, 500);
            
            
            if (numRings == 4) {
                fd(0.75, 0, 1);
                drift(-0.5, 1000);
                lock.setPosition(1);
                drift(0.5, 1000);
            } else if (numRings == 2) {
                fd(0.75, 0, 0.75);
                drift(-0.5, 500);
                lock.setPosition(1);
                drift(0.5, 500);
            } else {
                fd(0.75, 0, 0.5);
                drift(-0.5, 1000);
                lock.setPosition(1);
                drift(0.5, 1000);
            }
            
            
            // take back wobble goal
            while (pivot.getCurrentPosition() > 50) {
                pivot.setPower(-0.5);
            }
            pivot.setPower(0);
            
        }
    }
    
    private void initVuforia() {
        
        // * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    
    // Someone else finished this function so I'm just going to leave it like this...
    // private int selectiveCountRings(double seconds = 3, double certainty = .7, double zeroCertainty = .2) {
    //     double[] arr;
    //     double bestNum = -1, bestBestConf = certainty/2+zeroCertainty/2;
    //     double start = getRuntime();
    //     while (getRuntime() - start < seconds && bestBestConf < certainty && bestBestConf > zeroCertainty) {
    //         arr = countRings();
    //         if (arr[0] == 0) {
                
    //         } 
    //     }
    //     return 0;
    // }
    
    private int countRings() {
        int numRings = -1;
        int bestConf = -1;
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            // Run through updatedRecognitions, add telemetry data, and get best one
            for (int j = 0; j<updatedRecognitions.size(); j++) {
                Recognition recognition = updatedRecognitions.get(j);
                if (recognition.getConfidence() > .5 && (bestConf == -1 || recognition.getConfidence() > updatedRecognitions.get(bestConf).getConfidence()))
                    bestConf = j;
                telemetry.addData(String.format("label (%d)", j), recognition.getLabel());
                telemetry.addData("confidence", recognition.getConfidence());
                telemetry.addData(String.format("  left,top (%d)", j), "%.03f , %.03f",
                                  recognition.getLeft(), recognition.getTop());
                telemetry.addData(String.format("  right,bottom (%d)", j), "%.03f , %.03f",
                        recognition.getRight(), recognition.getBottom());
            }
            telemetry.addData("best confidence recognition", bestConf);
            telemetry.update();
        }
        if (bestConf == -1 || updatedRecognitions.get(bestConf).getConfidence() <= .2)
            numRings = 0;
        else 
            switch (updatedRecognitions.get(bestConf).getLabel()) {
                case "Single" :
                    numRings = 1;
                case "Quad" :
                    numRings = 4;
                    break;
                default :
                    break;
            }
        telemetry.addData("numRings", numRings);
        
        //double[] arr = {numRings, updatedRecognitions.get(bestConf).getConfidence()};
        return numRings;
    }
    
    private void launcher() {
        double start_time = getRuntime();
        ramp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (ramp.getCurrentPosition() < 600) {
            ramp.setPower(0.25);
            ramp_rings1.setPower(1);
            
        }
        ramp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        angle_adjust.setPosition(0);
        ramp.setPower(-0.2);
        sleep(200);
        while (getRuntime()-start_time < 5) {
            ramp.setPower(0.05);
            ramp_rings1.setPower(-1);
            ramp_rings2.setPower(-1);
        }
        angle_adjust.setPosition(1);
        while (ramp.getCurrentPosition() > 0) {
            ramp.setPower(-0.5);
        }
        ramp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ramp.setPower(0.05);
        ramp_rings1.setPower(0);
        ramp_rings2.setPower(0);
    }
    
    
    
    private void fd(double power, double angle, double interval) {
        double start_time = getRuntime();
        double curr_angle;
        while (getRuntime() - start_time < interval) {
          curr_angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
          if (curr_angle-angle > 5) {
            drive(power,power/2,0);
          } else if (curr_angle-angle < -5) {
            drive(power,-power/2,0);
          } else {
            drive(power,0,0);
          }
        }
        drive(-power/4,0,0);
        sleep(250);
        brake(250);
    }
      
    private void yaw(double power, double angle) {
        double curr_angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        if (angle!=180){
          while (Math.abs(curr_angle-angle)>3) {
            curr_angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            drive(0,power,0);
          }
        } else {
          while (Math.abs(angle-Math.abs(curr_angle))>3) {
            curr_angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            drive(0,power,0);
          }
        }
        brake(250);
    }
      
    private void drift(double power, long dur) {
        drive(0,0,power);
        sleep(dur);
        drive(0,0,-power);
        sleep(250);
        brake(250);
    }
      
    private void brake(long dur) {
        drive(0,0,0);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sleep(dur);
    }
    
    
    
    
    
    private void drive(double fd, double yaw, double drift) {
        if (Math.abs(yaw)<0.15) {
            yaw = 0;
        }
        if (Math.abs(drift)<0.15) {
            drift = 0;
        }
        double p1 = fd + yaw + drift;
        double p2 = fd - yaw - drift;
        double p3 = fd - yaw + drift;
        double p4 = fd + yaw - drift;
        
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
