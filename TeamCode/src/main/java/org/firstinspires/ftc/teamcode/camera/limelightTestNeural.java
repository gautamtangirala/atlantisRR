package org.firstinspires.ftc.teamcode.camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.Objects;


@Config
@TeleOp
public class limelightTestNeural extends LinearOpMode {

    private DcMotorEx horizSlides;
    private DcMotorEx vertSlides;
    public Limelight3A limelight;
    public Servo wrist;


    public static double horizP = 0.02;
    public static double horizI = 0;
    public static double horizD = 0.0002;
    public static double horizF = 0.00016;
    private static final PIDFController horizSlidePIDF = new PIDFController(horizP, horizI, horizD, horizF);
    public static double horizSetPoint = 0;
    public static double horizMaxPowerConstant = 1.0;
    int horizMotorPosition;
    public double ticksPerY = 15;







    @Override
    public void runOpMode() {
        // Initialize motors
        horizSlides = hardwareMap.get(DcMotorEx.class, "horizSlide");

        vertSlides = hardwareMap.get(DcMotorEx.class, "vertSlide");

        wrist = hardwareMap.get(Servo.class, "intakeClawWrist");

        vertSlides.setDirection(DcMotorSimple.Direction.REVERSE);


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(1);
        limelight.start();
        limelight.pipelineSwitch(2);

        // Set motors to use encoder mode
        horizSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        vertSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Connect to FTC Dashboard
        FtcDashboard horizDashboard = FtcDashboard.getInstance();
        LLResult result = limelight.getLatestResult();


        waitForStart();

        while (opModeIsActive()) {
            // Get latest Limelight results
            result = limelight.getLatestResult();
            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();

            // Define source (pixel) and destination (real-world) points
            Point[] srcPoints = new Point[4];
            srcPoints[0] = new Point(250, 90);
            srcPoints[1] = new Point(68, 86);
            srcPoints[2] = new Point(9, 240);
            srcPoints[3] = new Point(320, 240);

            Point[] dstPoints = new Point[4];
            dstPoints[0] = new Point(0, 10.625);
            dstPoints[1] = new Point(11.375, 10.625);
            dstPoints[2] = new Point(11.375, 0);
            dstPoints[3] = new Point(0, 0);

            // Convert points to MatOfPoint2f
            MatOfPoint2f srcMat = new MatOfPoint2f(srcPoints);
            MatOfPoint2f dstMat = new MatOfPoint2f(dstPoints);

            // Compute perspective transform matrix
            Mat transformMatrix = Imgproc.getPerspectiveTransform(srcMat, dstMat);

            LLResultTypes.DetectorResult closestSample = null;
            double minDistance = Double.MAX_VALUE;
            Point targetPoint = new Point(160, 160); // Target location

            for (LLResultTypes.DetectorResult detection : detections) {
                if (Objects.equals(detection.getClassName(), "yellow")) {
                    // Compute Euclidean distance (prioritizing X proximity)
                    double xWeight = 2.0; // Adjust weight as needed
                    double distance = Math.sqrt(
                            Math.pow((detection.getTargetXPixels() - targetPoint.x) * xWeight, 2) +
                                    Math.pow(detection.getTargetYPixels() - targetPoint.y, 2)
                    );

                    // Find the closest yellow sample
                    if (distance < minDistance) {
                        minDistance = distance;
                        closestSample = detection;
                    }
                }
            }

            if (closestSample != null) {
                // Get pixel coordinates of closest yellow object
                double x = closestSample.getTargetXPixels();
                double y = closestSample.getTargetYPixels();

                // Convert to Mat for perspective transformation
                Mat pointMat = new Mat(1, 1, CvType.CV_32FC2);
                pointMat.put(0, 0, new float[]{(float) x, (float) y});
                Mat resultMat = new Mat();

                // Apply perspective transform
                Core.perspectiveTransform(pointMat, resultMat, transformMatrix);
                float[] robotCoords = new float[2];
                resultMat.get(0, 0, robotCoords);

                // Extract transformed real-world coordinates
                double realX = robotCoords[0];
                double realY = robotCoords[1];

                // Determine if the detected object is vertical
                boolean vertical = isVertical(closestSample.getTargetCorners());

                // Print results to telemetry
                telemetry.addData("Closest Yellow Sample (Pixels)", "X: " + x + ", Y: " + y);
                telemetry.addData("Coordinates With X shifted", "X: " + (x - Math.tan(Math.toRadians(50))*y) + ", Y: " + y);
                telemetry.addData("Transformed Robot Coordinates", "X: " + realX + ", Y: " + realY);
                telemetry.addData("Is Vertical?", vertical);
            } else {
                telemetry.addData("Status", "No yellow samples detected.");
            }

            // Display current position of horizontal slides
            telemetry.addData("Current Position", horizSlides.getCurrentPosition());
            telemetry.update();
        }

    }


    // Function to determine orientation
    public  Boolean isVertical(List<List<Double>> points) {
        if (points.isEmpty()) {
            return true; // Return true if there's no valid data
        }

        double minX = points.stream().mapToDouble(p -> p.get(0)).min().orElse(Double.NaN);
        double maxX = points.stream().mapToDouble(p -> p.get(0)).max().orElse(Double.NaN);
        double minY = points.stream().mapToDouble(p -> p.get(1)).min().orElse(Double.NaN);
        double maxY = points.stream().mapToDouble(p -> p.get(1)).max().orElse(Double.NaN);


        if (Double.isNaN(minX) || Double.isNaN(maxX) || Double.isNaN(minY) || Double.isNaN(maxY)) {
            return true; // If for some reason values are invalid, assume vertical
        }

        // Calculate the difference in coordinates between the top-left and bottom-right
        double deltaX = maxX - minX;
        double deltaY = maxY - minY;

        return (maxY - minY) >= (maxX - minX); // True if vertical, False if horizontal
    }

}
