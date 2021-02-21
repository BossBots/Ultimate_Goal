package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

// image recognition class for robot
public class ImageRecognition {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY = "ASXuYar/////AAABmXFF9U0Sqkf4nhGpOxAwL4hjOruKN+pzxoY06iPyjRCJzC2VJLEcTeWGlru0xqBeYD1/4Q/Re8WeD61/uoVn4xHD2U4ZJcxUOgBIJ9tpv181fPxonQEECTo6DAbx6VaADyWN5deZ5fkOVeQD7He5kCgL7DA5VbYDsXCNoqF4Ifnj+pyVUlYZeuiIvpHUDGXfa6E5a3jJk4p6ksFK0BCObGsRtKfFsCKZvjz8shWT0ifp4majfLUu3J8xLNmEM6pLy9bsDWgANt+Ao+zrFDJ1jUGG92oI1nllBYQCW/PC3to0UeGPIeUWpTSFBZFp8GSAf633CgKcxSftde0rgBxMlrB2YIWeM6bPXPwbqSpvS/yT";


    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    private double maxConfidence;
    private String bestRecognition;

    // constructor
    public ImageRecognition() {
        maxConfidence = 0;
        bestRecognition = "Zero";
    }

    public void initializeSystem() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }
    }

    // getRecognition to return the recognition with the highest confidence
    public String getRecognition() {

        if (tfod != null) {

            // getUpdatedRecognitions() will return null if no new information is available since the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {



                bestRecognition = "Zero";
                maxConfidence = 0;

                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getConfidence() > maxConfidence) {
                        maxConfidence = recognition.getConfidence();
                        bestRecognition = recognition.getLabel();
                    }
                }
            }
            return bestRecognition;
        }
        return "Zero";
    }

    // close method to shut down TensorFlow
    public void close() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    // Initialize the Vuforia localization engine.
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

     // Initialize the TensorFlow Object Detection engine.
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
