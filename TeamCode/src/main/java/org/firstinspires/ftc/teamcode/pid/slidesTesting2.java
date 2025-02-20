package org.firstinspires.ftc.teamcode.pid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp
public class SlidesTesting2 extends LinearOpMode {

    private DcMotorEx horizSlides;
    private DcMotorEx vertSlides;

    public static double horizP = 0.02;
    public static double horizI = 0;
    public static double horizD = 0.0002;
    public static double horizF = 0.00016;
    private static final PIDFController horizSlidePIDF = new PIDFController(horizP, horizI, horizD, horizF);
    public static double horizSetPoint = 0;
    public static double horizMaxPowerConstant = 1.0;
    int horizMotorPosition;



    public static double vertP = 0.035;
    public static double vertI = 0.00002;
    public static double vertD = 0.0003;
    public static double vertF = 0.00018;
    private static final PIDFController vertSlidePIDF = new PIDFController(vertP, vertI, vertD, vertF);
    public static double vertSetPoint = 0;
    public static double vertMaxPowerConstant = 1.0;
    int vertMotorPosition;


    @Override
    public void runOpMode() {
        // Initialize motors
        horizSlides = hardwareMap.get(DcMotorEx.class, "horizSlide");

        vertSlides = hardwareMap.get(DcMotorEx.class, "vertSlide");
        vertSlides.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motors to use encoder mode
        horizSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        vertSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Connect to FTC Dashboard
        FtcDashboard horizDashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {
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


            vertMotorPosition = vertSlides.getCurrentPosition();

            vertSlidePIDF.setP(vertP);
            vertSlidePIDF.setI(vertI);
            vertSlidePIDF.setD(vertD);
            vertSlidePIDF.setF(vertF);

            vertSlidePIDF.setSetPoint(vertSetPoint);
            vertSlidePIDF.setTolerance(5);

            double vertMaxPower = (vertF * vertMotorPosition) + vertMaxPowerConstant;
            double vertPower = Range.clip(vertSlidePIDF.calculate(vertMotorPosition, vertSetPoint), (vertSetPoint > 50) ? -1.0 : -0.7, vertMaxPower);

            vertSlides.setPower(vertPower);

            telemetry.addData("Target Position", horizSetPoint);
            telemetry.addData("Current Position", horizSlides.getCurrentPosition());
            telemetry.addData("Error", Math.abs(horizSetPoint - horizSlides.getCurrentPosition()));
            telemetry.addData("Power", horizPower);
            telemetry.addLine();
            telemetry.addData("Target Position", vertSetPoint);
            telemetry.addData("Current Position", vertSlides.getCurrentPosition());
            telemetry.addData("Error", Math.abs(vertSetPoint - vertSlides.getCurrentPosition()));
            telemetry.addData("Power", vertPower);
            telemetry.update();
        }
    }
}
