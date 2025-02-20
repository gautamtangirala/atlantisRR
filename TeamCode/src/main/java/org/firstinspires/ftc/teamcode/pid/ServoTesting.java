package org.firstinspires.ftc.teamcode.pid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp
public class ServoTesting extends LinearOpMode {

    public Servo intakeClawWrist;
    public Servo intakeClawTilt;
    public Servo intakeTransfer;
    public Servo depositTransfer;



    public static double intakeClawWristPosition;
    public static double intakeClawTiltPosition;
    public static double intakeTransferPosition;
    public static double depositTransferPosition;





    @Override
    public void runOpMode() {
        // Initialize motors

        intakeClawTilt = hardwareMap.get(Servo.class, "intakeClawTilt");
        intakeClawTilt.setDirection(Servo.Direction.REVERSE);
        intakeClawWrist = hardwareMap.get(Servo.class, "intakeClawWrist");
        intakeTransfer = hardwareMap.get(Servo.class, "intakeTransfer");


        depositTransfer = hardwareMap.get(Servo.class, "depositTransfer");

        FtcDashboard horizDashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {
            intakeClawWrist.setPosition(intakeClawWristPosition);
            intakeClawTilt.setPosition(intakeClawTiltPosition);
            intakeTransfer.setPosition(intakeTransferPosition);
            depositTransfer.setPosition(depositTransferPosition);
        }
    }
}
