package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;
import org.firstinspires.inspection.InspectionState;
import org.openftc.easyopencv.OpenCvWebcam;

/*
 * This is meant to be a library!
 * Do not copy and edit the code
 * Import it in a new java class!
 *
 * DO NOT MODIFY
 */

public class atlantisAutoEssentials extends LinearOpMode {
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

    public int tickRotation;
    public int tickAcceptance;

    public OpenCvWebcam webcam;


    //Variables
    double depositClawClose = 0.435;
    double depositClawOpen = 0.1;

    double depositTransferIn = 0.15;
    double depositTransferOut = 1;

    double intakeClawClose = 0.525;
    double intakeClawOpen = 0;

    double intakeTransferOut = 0.975;
    double intakeTransferIn = 0.4;

    int highRungHeight = 570;
    int highBasketHeight = 2500;

    double intakeWristVert = 0.525;
    double intakeWristHoriz = 0;

    double turnMulti = 0.6;
    double slowedDownMulti = 0.8;

    boolean specimenMode = true;
    boolean sampleMode = false;




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












    public void intakeGrab(double pos){
        intakeClawGrabLeft.setPosition(pos);
        intakeClawGrabRight.setPosition(pos);
    }

    public void depositGrab(double pos){
        intakeClawGrabLeft.setPosition(pos);
        intakeClawGrabRight.setPosition(pos);
    }

    public Action openDeposit() {
        return new InstantAction(() -> { depositGrab(depositClawOpen); });
    }

    public Action closeDeposit() {
        return new InstantAction(() -> { depositGrab(depositClawClose); });
    }

    public Action openIntake() {
        return new InstantAction(() -> { intakeGrab(intakeClawOpen); });
    }

    public Action closeIntake() {
        return new InstantAction(() -> { intakeGrab(intakeClawClose); });
    }





    public Action placeSpecimen(){
        return new InstantAction(() -> {
        depositTransfer.setPosition(0.8);}
        );
    }

    public void setupTransfer(){

        closeIntake();
        openDeposit();
        intakeTransfer.setPosition(0.5);
        intakeClawTilt.setPosition(1);
        depositTransfer.setPosition(0);
    }




    public void transferToDep(){
        closeDeposit();
        openIntake();
        intakeTransfer.setPosition(0.3);
        depositTransfer.setPosition(0.5);
    }






    public Action moveSlideTop(){
        return new InstantAction(() -> {
            vertSlides.setTargetPosition(highBasketHeight);
        vertSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        vertSlides.setPower(1);  });
    }

    public Action moveSlideSpeci(){
        return new InstantAction(() -> {
            depositTransfer.setPosition(0.5);
            vertSlides.setTargetPosition(highRungHeight);
            vertSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            vertSlides.setPower(1);  });
    }

    public Action moveSlideMid(){
        return new InstantAction(() -> {
            vertSlides.setTargetPosition(250);
            vertSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            vertSlides.setPower(1); });
    }

    public Action moveSlideBottom(){
        return new InstantAction(() -> {
            depositTransfer.setPosition(depositTransferOut);
            vertSlides.setTargetPosition(5);
            vertSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            vertSlides.setPower(1); });
    }


/*
    public void runCamera(){
        final String webcamName = "Webcam 1";
        final int length = 320;
        final int width = 240;
        final int squareSize = 30;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);

        detector = new PPR1SleeveDetector(telemetry, length, width, squareSize, 30, -56);
        webcam.setPipeline(detector);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(length, width, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

    }
    public void stopCamera(){webcam.stopStreaming();}
*/




    @Override
    public void runOpMode(){
    }
}

