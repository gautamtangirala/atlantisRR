package org.firstinspires.ftc.teamcode.opmodes.auto;

import static java.lang.Math.round;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

/*
 * This is meant to be a library!
 * Do not copy and edit the code
 * Import it in a new java class!
 *
 * DO NOT MODIFY
 */

@Config
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

    public Limelight3A limelight;
    public LLResult result;
    List<LLResultTypes.ColorResult> colorResults;

    public Servo depositTransfer;
    public Servo depositClawGrabRight;
    public Servo depositClawGrabLeft;

    public int tickRotation;
    public int tickAcceptance;

    public OpenCvWebcam webcam;


    //Variables
    public double depositClawClose = 0.45;
    public double depositClawOpen = 0.1;

    public double depositTransferIn = 0.1725;
    public double depositTransferOut = 1;
    public double slamSpeciPos = 1;

    public double intakeClawClose = 0.45;
    public double intakeClawOpen = 0.1;

    public int horizTransferPos = 34;
    public int horizOutPos = 470;

    public double intakeTransferOut = 1;
    public double intakeTransferIn = 0.4;

    public int highRungHeight = 525;
    public int highBasketHeight = 950;
    public int speciPickupHeight = 26;

    public double intakeWristVert = 0.525;
    public double intakeWristHoriz = 0;

    public double turnMulti = 0.6;
    public double slowedDownMulti = 0.8;


    public void initDrive() {
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

    public void initLimelight(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public void initSubsystems() {
        vertSlides = hardwareMap.get(DcMotorEx.class, "vertSlide");
        vertSlides.setDirection(DcMotorSimple.Direction.REVERSE);
        vertSlides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        vertSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        vertSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        horizSlides = hardwareMap.get(DcMotorEx.class, "horizSlide");
        horizSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        intakeClawTilt = hardwareMap.get(Servo.class, "intakeClawTilt");
        intakeClawTilt.setDirection(Servo.Direction.REVERSE);
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


    public void intakeGrab(double pos) {
        intakeClawGrabLeft.setPosition(pos);
        intakeClawGrabRight.setPosition(pos);
    }

    public void depositGrab(double pos) {
        depositClawGrabRight.setPosition(pos);
        depositClawGrabLeft.setPosition(pos);
    }

    public Action openDeposit() {
        return new InstantAction(() -> {
            depositGrab(depositClawOpen);
        });
    }


    public Action closeDeposit() {
        return new InstantAction(() -> {
            depositGrab(depositClawClose);
        });
    }

    public Action openIntake() {
        return new InstantAction(() -> {
            intakeGrab(intakeClawOpen);
        });
    }

    public Action closeIntake() {
        return new InstantAction(() -> {
            intakeGrab(intakeClawClose);
        });
    }


    public Action depositTransferAction(double pos) {
        return new InstantAction(() -> {
            depositTransfer.setPosition(pos);
        });
    }

    public Action intakeTransferAction(double pos) {
        return new InstantAction(() -> {
            intakeTransfer.setPosition(pos);
        });
    }


    public Action placeSpecimen() {
        return new InstantAction(() -> {
            depositTransfer.setPosition(slamSpeciPos);
        }
        );
    }


    public Action moveSlideHighBasket() {

        return new Action() {


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                vertSetPoint = highBasketHeight;


                return Math.abs(vertSlides.getCurrentPosition() - highBasketHeight) > 10;
            }
        };
    }

    public Action moveSlideHighRung() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                depositTransfer.setPosition(depositTransferOut);
                vertSetPoint = highRungHeight;

                return Math.abs(vertSlides.getCurrentPosition() - highRungHeight) > 5;
            }
        };
    }


    public Action placeSpecimenSlides() {
        return new Action() {

            int offset = highRungHeight - 142;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                depositTransfer.setPosition(depositTransferOut);
                vertSetPoint = offset;

                return Math.abs(vertSlides.getCurrentPosition() - offset) > 5;
            }
        };
    }


    public Action moveSlideTop() {
        return new InstantAction(() -> {
            vertSlides.setTargetPosition(highBasketHeight);
            vertSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            vertSlides.setPower(1);
        });
    }

    public Action moveSlideSpeci() {
        return new InstantAction(() -> {
            depositTransfer.setPosition(0.5);
            vertSlides.setTargetPosition(highRungHeight);
            vertSlides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            vertSlides.setPower(1);
        });
    }


    public Action moveSlideBottom() {
        return new InstantAction(() -> {
            vertSetPoint = 0;
        });
    }

    public Action moveSlideSpeciPickup() {
        return new InstantAction(() -> {
            vertSetPoint = speciPickupHeight;
        });
    }

    public Action moveSlidePark() {
        return new InstantAction(() -> {
            vertSetPoint = 190;
        });
    }


    public Action horizSlideOut(boolean sub) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                openIntake();
                intakeTransfer.setPosition(sub ? 0.75 : 0.9);
                intakeClawTilt.setPosition(0.05);
                horizSetPoint = horizOutPos;

                return Math.abs(horizSlides.getCurrentPosition() - horizOutPos) > 10;
            }
        };
    }

    public Action horizSlideOut(boolean sub, int pos) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                openIntake();
                intakeTransfer.setPosition(sub ? 0.75 : 0.9);
                intakeClawTilt.setPosition(0.05);
                horizSetPoint = pos;

                return Math.abs(horizSlides.getCurrentPosition() - pos) > 10;
            }
        };
    }


    public Action moveWrist(double pos) {
        return new InstantAction(() -> {
            intakeClawWrist.setPosition(pos);
        });
    }


    public Action clawPickup() {


        return new InstantAction(() -> {
            openIntake();
            depositTransfer.setPosition(depositTransferIn);
            openDeposit();
            intakeTransfer.setPosition(intakeTransferOut);
            intakeClawTilt.setPosition(0.1);
        });


    }


    public Action transferSensed(int state) {

        if (state == 1) {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    intakeTransfer.setPosition(intakeTransferIn);
                    intakeClawTilt.setPosition(0.85);
                    intakeClawWrist.setPosition(intakeWristVert);

                    depositTransfer.setPosition(depositTransferIn);
                    openDeposit();

                    vertSetPoint = 0;

                    horizSetPoint = ((int) (horizTransferPos * 2.5));
                    return Math.abs(horizSlides.getCurrentPosition() - horizTransferPos * 2.5) > 5;
                }
            };
        } else if (state == 2) {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    horizSetPoint = horizTransferPos;
                    return Math.abs(horizSlides.getCurrentPosition() - horizTransferPos) > 5;
                }
            };
        } else {
            return null;
        }
    }

    public Action transfer(int state) {
        if (state == 1) {
            return new InstantAction(() -> {
                intakeTransfer.setPosition(intakeTransferIn);
                intakeClawTilt.setPosition(0.85);
                intakeClawWrist.setPosition(intakeWristVert);

                depositTransfer.setPosition(depositTransferIn);
                openDeposit();

                vertSetPoint = 0;



                horizSetPoint = horizTransferPos;

            });
        } else if (state == 2) {
            return new InstantAction(() -> {
                horizSetPoint = horizTransferPos;
            });
        } else if (state == 3) {
            return new InstantAction(() -> {
                depositTransfer.setPosition(0.5);
                intakeTransfer.setPosition(intakeTransferIn + 0.1);
            });
        } else {
            return null;
        }
    }






    public static double vertP = 0.035;
    public static double vertI = 0.00002;
    public static double vertD = 0.0003;
    public static double vertF = 0.00018;
    private static final PIDFController vertSlidePIDF = new PIDFController(vertP, vertI, vertD, vertF);
    public static double vertSetPoint = 0;
    public static double vertMaxPowerConstant = 1.0;
    int vertMotorPosition;


    public static double horizP = 0.025;
    public static double horizI = 0;
    public static double horizD = 0.0002;
    public static double horizF = 0.0001;
    private static final PIDFController horizSlidePIDF = new PIDFController(horizP, horizI, horizD, horizF);
    public static double horizSetPoint = 0;
    public static double horizMaxPowerConstant = 0.8;
    int horizMotorPosition;

    // Run in parallel w auto
    public Action updatePidAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
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

                telemetry.addData("Ticks to Block", ticksToBlock);
                telemetry.addData("inchesToBlock", inchesToBlock);
                telemetry.addData("isVertical", vertical);
                telemetry.addLine();
                telemetry.addData("Horiz Target Position", horizSetPoint);
                telemetry.addData("Horiz Current Position", horizSlides.getCurrentPosition());
                telemetry.addData("Horiz Error", Math.abs(horizSetPoint - horizSlides.getCurrentPosition()));
                telemetry.addData("Horiz Power", horizPower);
                telemetry.addLine();
                telemetry.addData("Vert Target Position", vertSetPoint);
                telemetry.addData("Vert Current Position", vertSlides.getCurrentPosition());
                telemetry.addData("Vert Error", Math.abs(vertSetPoint - vertSlides.getCurrentPosition()));
                telemetry.addData("Vert Power", vertPower);
                telemetry.update();

                return true;
            }
        };
    }





    public Action updateLimelight(){
        return new InstantAction(() -> {
            result = limelight.getLatestResult();
            colorResults = result.getColorResults();
            ticksToBlock = (int) (Math.round(result.getTy()) * 15);
            inchesToBlock = (-0.298642*(result.getTx()))-0.11495;
            vertical = colorResults.stream().map(colorTarget -> isVertical(colorTarget.getTargetCorners())).findFirst().orElse(true);
            clawPosBlock = vertical ? intakeWristVert : intakeWristHoriz;
        });
    }

    public static Boolean isVertical(List<List<Double>> points) {
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

        return (maxY - minY) >= (maxX - minX); // True if vertical, False if horizontal
    }

    public static int ticksToBlock = 480;
    public static double inchesToBlock = 0;
    public static boolean vertical = true;
    public static double clawPosBlock = 0.525;






    @Override
    public void runOpMode(){
    }


}

