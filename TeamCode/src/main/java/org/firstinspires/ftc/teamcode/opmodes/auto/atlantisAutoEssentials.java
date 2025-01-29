package org.firstinspires.ftc.teamcode.opmodes.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    public Servo intakeClawWrist;

    public Servo depositTransfer;
    public Servo depositClawGrabRight;
    public Servo depositClawGrabLeft;

    public int tickRotation;
    public int tickAcceptance;

    public OpenCvWebcam webcam;


    //Variables
    public  double depositClawClose = 0.45;
    public double depositClawOpen = 0.1;

    public double depositTransferIn = 0.1725;
    public double depositTransferOut = 1;
    public double slamSpeciPos = 1;

    public double intakeClawClose = 0.45;
    public  double intakeClawOpen = 0.1;

    public int horizTransferPos = 110;
    public int horizOutPos = 1240;

    public double intakeTransferOut = 1;
    public double intakeTransferIn = 0.4;

    public int highRungHeight = 500;
    public int highBasketHeight = 960;
    public int speciPickupHeight = 26;

    public  double intakeWristVert = 0.525;
    public double intakeWristHoriz = 0;

    public double turnMulti = 0.6;
    public  double slowedDownMulti = 0.8;






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


        intakeClawTilt = hardwareMap.get(Servo.class, "intakeClawTilt");
        intakeClawWrist = hardwareMap.get(Servo.class, "intakeClawWrist");
        intakeTransfer = hardwareMap.get(Servo.class, "intakeTransfer");
        intakeClawGrabLeft = hardwareMap.get(Servo.class, "iCGL");
        intakeClawGrabRight = hardwareMap.get(Servo.class, "iCGR");
        intakeClawGrabRight.setDirection(Servo.Direction.REVERSE);

        depositTransfer = hardwareMap.get(Servo.class, "depositTransfer");
        depositClawGrabLeft = hardwareMap.get(Servo.class, "dCGL");
        depositClawGrabRight = hardwareMap.get(Servo.class, "dCGR");
        depositClawGrabLeft.setDirection(Servo.Direction.REVERSE);

    }












    public void intakeGrab(double pos){
        intakeClawGrabLeft.setPosition(pos);
        intakeClawGrabRight.setPosition(pos);
    }

    public void depositGrab(double pos){
        depositClawGrabRight.setPosition(pos);
        depositClawGrabLeft.setPosition(pos);
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


    public Action depositTransferAction(double pos) {
        return new InstantAction(() -> { depositTransfer.setPosition(pos); });
    }

    public Action intakeTransferAction(double pos) {
        return new InstantAction(() -> { intakeTransfer.setPosition(pos); });
    }



    public Action placeSpecimen(){
        return new InstantAction(() -> {
        depositTransfer.setPosition(slamSpeciPos);}
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

    public Action moveSlideHighBasket() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                vertSlides.setTargetPosition(highBasketHeight);
                vertSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                vertSlides.setPower(1);

                return vertSlides.getCurrentPosition() < highBasketHeight - 10;
            }
        };
    }

    public Action moveSlideHighRung() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                depositTransfer.setPosition(depositTransferOut);
                vertSlides.setTargetPosition(highRungHeight);
                vertSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                vertSlides.setPower(1);

                return Math.abs(vertSlides.getCurrentPosition() - highRungHeight) > 5;
            }
        };
    }


    public Action placeSpecimenSlides() {
        return new Action() {

            int offset = highRungHeight - 120;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                depositTransfer.setPosition(depositTransferOut);
                vertSlides.setTargetPosition(offset);
                vertSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                vertSlides.setPower(1);

                return Math.abs(vertSlides.getCurrentPosition() - offset) > 5;
            }
        };
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


    public Action moveSlideBottom(){
        return new InstantAction(() -> {
            depositTransfer.setPosition(0.5);
            vertSlides.setTargetPosition(5);
            vertSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            vertSlides.setPower(1); });
    }

    public Action moveSlideSpeciPickup(){
        return new InstantAction(() -> {
            depositTransfer.setPosition(0.5);
            vertSlides.setTargetPosition(speciPickupHeight);
            vertSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            vertSlides.setPower(1); });
    }

    public Action moveSlidePark(){
        return new InstantAction(() -> {
            depositTransfer.setPosition(0.5);
            vertSlides.setTargetPosition(151);
            vertSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            vertSlides.setPower(1); });
    }






    public Action horizSlideOut(boolean sub) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                openIntake();
                intakeTransfer.setPosition(sub ? 0.75 : 0.9);
                intakeClawTilt.setPosition(0.05);
                horizSlides.setTargetPosition(horizOutPos);
                horizSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                horizSlides.setPower(1);

                return Math.abs(horizSlides.getCurrentPosition() - horizOutPos) > 10 ;
            }
        };
    }





    public Action moveWrist(double pos){
        return new InstantAction(() ->{
            intakeClawWrist.setPosition(pos);
        });
    }


    public Action clawPickup(){


        return new InstantAction(() -> {
            openIntake();
            depositTransfer.setPosition(depositTransferIn);
            openDeposit();
            intakeTransfer.setPosition(intakeTransferOut);
            intakeClawTilt.setPosition(0.075);
        });


    }


    public Action transferSensed(int state){

        if(state == 1) {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    intakeTransfer.setPosition(intakeTransferIn);
                    intakeClawTilt.setPosition(1);
                    intakeClawWrist.setPosition(intakeWristVert);

                    depositTransfer.setPosition(depositTransferIn);
                    openDeposit();

                    vertSlides.setTargetPosition(0);
                    vertSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    vertSlides.setPower(1);

                    horizSlides.setTargetPosition((int)(horizTransferPos * 2.5));
                    horizSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    horizSlides.setPower(1);
                    return Math.abs(horizSlides.getCurrentPosition() - horizTransferPos * 2.5) > 5;
                }
            };
        }
        else if (state == 2){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    horizSlides.setTargetPosition(horizTransferPos);
                    horizSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    horizSlides.setPower(1);
                    return Math.abs(horizSlides.getCurrentPosition() - horizTransferPos) > 5 ;
                }
            };
        }
        else {
            return null;
        }
    }

    public Action transfer(int state){
        if (state == 1){
            return new InstantAction(() ->{
                intakeTransfer.setPosition(intakeTransferIn);
                intakeClawTilt.setPosition(1);
                intakeClawWrist.setPosition(intakeWristVert);

                depositTransfer.setPosition(depositTransferIn);
                openDeposit();

                vertSlides.setTargetPosition(0);
                vertSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                vertSlides.setPower(1);

                horizSlides.setTargetPosition((int)(horizTransferPos * 2.5));
                horizSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                horizSlides.setPower(1);
            });
        } else if (state == 2) {
            return new InstantAction(() ->{
                horizSlides.setTargetPosition(horizTransferPos);
                horizSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                horizSlides.setPower(1);
            });
        } else if (state == 3) {
            return new InstantAction(() -> {
            depositTransfer.setPosition(0.5);
            intakeTransfer.setPosition(intakeTransferIn+0.1);});
        } else {
            return null;
        }
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

