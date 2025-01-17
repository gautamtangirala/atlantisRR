package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.easyopencv.OpenCvWebcam;




/*
* This is meant to be a library!
* Do not copy and edit the code
* Import it in a new java class!
*
* DO NOT MODIFY
*/

public class atlantisTeleopEssentials extends LinearOpMode {
    public DcMotorEx LF; // LeftFront Motor
    public DcMotorEx LB; // LeftBack Motor
    public DcMotorEx RF; // RightFront Motor
    public DcMotorEx RB; // RightBack Motor

    public DcMotorEx vertSlides;
    public DcMotorEx horizSlides;


    public Servo intakeClawGrabLeft;
    public Servo intakeClawGrabRight;
    public Servo intakeClawTilt;
    public Servo intakeTransfer;

    public Servo depositTransfer;
    public Servo depositClawGrabRight;
    public Servo depositClawGrabLeft;

    double vertical;
    double horizontal;
    double pivot;

    public int tickRotation;
    public int tickAcceptance;

    public OpenCvWebcam webcam;




    public void initDrive(){
        LF = hardwareMap.get(DcMotorEx.class, "leftFront");
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) LF).setTargetPositionTolerance(tickAcceptance);

        LB = hardwareMap.get(DcMotorEx.class, "leftRear");
        LB.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) LB).setTargetPositionTolerance(tickAcceptance);

        RF = hardwareMap.get(DcMotorEx.class, "rightFront");
        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        RF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) RF).setTargetPositionTolerance(tickAcceptance);

        RB = hardwareMap.get(DcMotorEx.class, "rightRear");
        RB.setDirection(DcMotorSimple.Direction.FORWARD);
        RB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) RB).setTargetPositionTolerance(tickAcceptance);

    }

    public void initSubsystems(){
        vertSlides = hardwareMap.get(DcMotorEx.class, "vertSlide");
        vertSlides.setDirection(DcMotorSimple.Direction.REVERSE);
        vertSlides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        vertSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        horizSlides = hardwareMap.get(DcMotorEx.class, "horizSlide");
        horizSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


//        intakeTransfer = hardwareMap.get(Servo.class, "intakeTransfer");
//
//        intakeClawGrabRight = hardwareMap.get(Servo.class, "iCGR");
//        intakeClawGrabLeft = hardwareMap.get(Servo.class, "iCGL");
        //intakeClawGrabRight.setDirection(Servo.Direction.REVERSE); // use in case needed


        depositTransfer = hardwareMap.get(Servo.class, "depositTransfer");

        depositClawGrabLeft = hardwareMap.get(Servo.class, "dCGL");
        depositClawGrabRight = hardwareMap.get(Servo.class, "dCGR");
        //depositClawGrabRight.setDirection(Servo.Direction.REVERSE); // use in case needed

    }
    


    public void gamepad1Controls() {
        // Initiating Holonomic Drive

        boolean slowMode = false;
        double SlowedDownMulti = 0.8;
        if(gamepad2.y) {

            slowMode = true;
        }
        else if (gamepad2.a){
            slowMode = false;
        }
        double turnMulti = 0.6;
        if (vertSlides.getCurrentPosition() > 200) {
            SlowedDownMulti = 0.6;
        }
        if(slowMode){
            turnMulti = 0.4;
        }
        else{
            turnMulti = 0.6;
        }
        vertical = gamepad1.right_stick_y  * SlowedDownMulti;
        horizontal = -gamepad1.right_stick_x  * SlowedDownMulti;
        pivot = -gamepad1.left_stick_x  * turnMulti;

        LF.setPower(pivot + vertical + horizontal);
        LB.setPower(pivot + vertical - horizontal );
        RF.setPower(0 - pivot + vertical - horizontal);
        RB.setPower(0 - pivot + vertical + horizontal);
/*
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
        ));
*/


    }

    public void clawControls(){
        if(gamepad2.x){
        }
        else if(gamepad2.b){

        }
        else if(gamepad2.a){


        }
        else if(gamepad2.y && vertSlides.getCurrentPosition()<300){

        }
        else if(gamepad2.right_bumper){


        }
        else if(gamepad2.left_bumper){


        }
    }

    public void slideControls() {


        if (gamepad2.dpad_left){

            vertSlides.setTargetPosition(200);
            vertSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            vertSlides.setPower(1);

            while (vertSlides.isBusy()){gamepad1Controls(); telemetry.addData("Slide Height",vertSlides.getCurrentPosition()); telemetry.update(); clawControls(); }

        }
        else if (gamepad2.dpad_up){
            vertSlides.setTargetPosition(645);
            vertSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            vertSlides.setPower(1);

            while (vertSlides.isBusy()){gamepad1Controls(); telemetry.addData("Slide Height",vertSlides.getCurrentPosition()); telemetry.update(); clawControls(); }
        }
        else if(gamepad2.dpad_right){
        }
        else if(gamepad2.dpad_down){


            vertSlides.setTargetPosition(10);
            vertSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            vertSlides.setPower(1);

            while (vertSlides.isBusy()){gamepad1Controls(); telemetry.addData("Slide Height",vertSlides.getCurrentPosition()); telemetry.update(); clawControls(); }
        }
        else{
            vertSlides.setPower(0);
        }



   

    }


    // smaller functions

    @Override
    public void runOpMode(){}
    }
