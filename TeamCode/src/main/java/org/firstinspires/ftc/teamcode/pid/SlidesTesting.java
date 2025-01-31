package org.firstinspires.ftc.teamcode.pid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;



@Disabled
@Config
@TeleOp(name = "Slides Test with Dashboard", group = "Test")
public class SlidesTesting extends LinearOpMode {

    // PID coefficients adjustable via FTC Dashboard
    public static double kP = 0;
    public static double kD = 0;
    public static double targetPosition = 0;

    private DcMotorEx vertSlides;

    @Override
    public void runOpMode() {
        // Initialize motors
        vertSlides = hardwareMap.get(DcMotorEx.class, "vertSlide");
        vertSlides.setDirection(DcMotorSimple.Direction.REVERSE);


        // Set motors to use encoder mode
        vertSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Connect to FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Variables for PD control
        double error;
        double prevError = 0;
        double derivative;
        double output;

        waitForStart();

        while (opModeIsActive()) {
            // Calculate error
            error = targetPosition - vertSlides.getCurrentPosition();

            // Calculate derivative
            derivative = error - prevError;

            // PD control output
            output = (kP * error) + (kD * derivative);

            // Apply power to motors
            vertSlides.setPower(output);

            // Update previous error
            prevError = error;

            // Send telemetry data to FTC Dashboard
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", vertSlides.getCurrentPosition());
            telemetry.addData("Error", error);
            telemetry.addData("P Term", kP * error);
            telemetry.addData("D Term", kD * derivative);
            telemetry.addData("Output", output);
            telemetry.update();
        }
    }


    
}
