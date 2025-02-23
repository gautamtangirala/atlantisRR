package org.firstinspires.ftc.teamcode.camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;


@Config
@TeleOp
public class limelightTest extends LinearOpMode {

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

        // Set motors to use encoder mode
        horizSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        vertSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Connect to FTC Dashboard
        FtcDashboard horizDashboard = FtcDashboard.getInstance();
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.ColorResult> colorResults = result.getColorResults();


        waitForStart();

        while (opModeIsActive()) {

            result = limelight.getLatestResult();
            colorResults = result.getColorResults();
            if (result != null && result.isValid()) {
                double tx = result.getTx(); // How far left or right the target is (degrees)
                double ty = result.getTy(); // How far up or down the target is (degrees)
                double ta = result.getTa();
                for (LLResultTypes.ColorResult colorTarget : colorResults) {
                    List<List<Double>> area = colorTarget.getTargetCorners();
                    wrist.setPosition(isVertical(area) ? 0.525 : 0);

                    //telemetry.addData("Is Vertical?", isVertical(area));

                }

                // How big the target looks (0%-100% of the image)

                telemetry.addData("Inches X", (-0.298642 * (result.getTx())) - 0.11495);
                telemetry.addLine();
                telemetry.addData("Target X", tx);

                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
            } else {
                telemetry.addData("Limelight", "No Targets");
            }

            horizSetPoint =  0.126217 * Math.pow(result.getTy(), 2) + 9.12149 * result.getTy() + 57.78645;
            horizMotorPosition = horizSlides.getCurrentPosition();

            horizSlidePIDF.setP(horizP);
            horizSlidePIDF.setI(horizI);
            horizSlidePIDF.setD(horizD);
            horizSlidePIDF.setF(horizF);

            horizSlidePIDF.setSetPoint(horizSetPoint);
            horizSlidePIDF.setTolerance(5);

            double horizMaxPower = (horizF * horizMotorPosition) + horizMaxPowerConstant;
            double horizPower = Range.clip(horizSlidePIDF.calculate(horizMotorPosition, horizSetPoint), -horizMaxPower, horizMaxPower);

            horizSlides.setPower(horizPower);



            telemetry.addData("Target Position", horizSetPoint);
            telemetry.addData("Current Position", horizSlides.getCurrentPosition());
            telemetry.addData("Error", Math.abs(horizSetPoint - horizSlides.getCurrentPosition()));
            telemetry.addData("Power", horizPower);
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

        // Calculate the angle of the diagonal relative to the x-axis using atan2 (which handles quadrant correctly)
        double angle = Math.toDegrees(Math.atan2(deltaY, deltaX)); // Convert radians to degrees

        // Adding the angle to telemetry
        telemetry.addData("Orientation Angle (Degrees)", angle);

        return (maxY - minY) >= (maxX - minX); // True if vertical, False if horizontal
    }

}
