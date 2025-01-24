package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "atlantisTele")
public class atlantisTele extends LinearOpMode {
    public DcMotorEx horizSlides;
    public DcMotorEx vertSlides;

    public Servo intakeClawWrist;
    public Servo intakeClawTilt;
    public Servo intakeTransfer;
    public Servo intakeClawGrabLeft;
    public Servo intakeClawGrabRight;
    public Servo depositTransfer;
    public Servo depositClawGrabLeft;
    public Servo depositClawGrabRight;
    public Servo depositTilt;

    public DcMotorEx LF, LB, RF, RB; // Drive motors


    //Variables
    double depositClawClose = 0.435;
    double depositClawOpen = 0.1;

    double depositTransferIn = 0.1725;
    double depositTransferOut = 1;
    double slamSpeciPos = 1;

    double intakeClawClose = 0.525;
    double intakeClawOpen = 0;

    double intakeTransferOut = 1;
    double intakeTransferIn = 0.4;

    int highRungHeight = 570;
    int highBasketHeight = 2500;

    int horizTransferPos = 110;

    double intakeWristVert = 0.525;
    double intakeWristHoriz = 0;

    double turnMulti = 0.6;
    double slowedDownMulti = 0.8;

    boolean specimenMode = true;
    boolean sampleMode = false;







    @Override
    public void runOpMode() {
        initMotors();
        initServos();

        waitForStart();

        while (opModeIsActive()) {
            // Gamepad 1: Holonomic drive
            holonomicDrive();

            if (specimenMode) {
                // Specimen Mode Controls
                if (gamepad2.dpad_up) {
                    highRung();
                } else if (gamepad2.dpad_right) {
                    if (slamState == SlamState.IDLE) {
                        startSlamSpecimen();
                    }
                } else if (gamepad2.dpad_left) {
                    specimenPickup();
                } else if (gamepad2.dpad_down) {
                    homePosition();
                } else if (gamepad2.left_bumper) {
                    startHorizontalSlidesOut();
                } else if (gamepad2.right_bumper) {
                    horizontalSlidesIn();
                }

                // Common Controls
                if (gamepad2.b) {
                    depositClaw(depositClawOpen);
                } else if (gamepad2.x) {
                    depositClaw(depositClawClose);
                } else if (gamepad2.a && pickupState == PickupState.IDLE) {
                    startPickupSample();
                } else if (gamepad2.y) {
                    startTransfer();
                }
            } else if (sampleMode) {
                // Sample Mode Controls
                if (gamepad2.dpad_up && transferState == TransferState.IDLE) {
                    startHighBasket();
                } else if (gamepad2.right_bumper && transferState == TransferState.IDLE) {
                    startTransfer();
                } else if (gamepad2.dpad_down) {
                    homePosition();
                } else if (gamepad2.left_bumper) {
                    startHorizontalSlidesOut();
                } else if (gamepad2.dpad_right) {
                    horizontalSlidesIn();
                }

                // Common Controls
                if (gamepad2.b) {
                    depositClaw(depositClawOpen);
                } else if (gamepad2.x) {
                    depositClaw(depositClawClose);
                } else if (gamepad2.a && pickupState == PickupState.IDLE) {
                    startPickupSample();
                } else if (gamepad2.y) {
                    intakeClaw(intakeClawOpen);
                }
            }

            // Mode Switching Controls
            if (gamepad1.a) {
                specimenMode = false;
                sampleMode = true;
            } else if (gamepad1.y) {
                specimenMode = true;
                sampleMode = false;
            }

            if (gamepad1.left_bumper) {
                intakeClawWrist.setPosition(intakeWristHoriz);
            } else if (gamepad1.right_bumper) {
                intakeClawWrist.setPosition(intakeWristVert);
            }



            //handle state machines
            handlePickupSample();
            handleSlamSpecimen();
            handleTransfer();
            handleHorizontalSlidesOut();
            handleHighBasket();

            // Telemetry
            telemetry.addData("MODE", specimenMode ? "SPECIMEN" : sampleMode ? "SAMPLE" : "NONE");
            telemetry.addData("vert slide position", vertSlides.getCurrentPosition());
            telemetry.addData("horizontal slide position", horizSlides.getCurrentPosition());

            telemetry.update();


        }


    }

    private void initMotors() {
        horizSlides = hardwareMap.get(DcMotorEx.class, "horizSlide");
        vertSlides = hardwareMap.get(DcMotorEx.class, "vertSlide");

        LF = hardwareMap.get(DcMotorEx.class, "leftFront");
        LB = hardwareMap.get(DcMotorEx.class, "leftRear");
        RF = hardwareMap.get(DcMotorEx.class, "rightFront");
        RB = hardwareMap.get(DcMotorEx.class, "rightRear");

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        horizSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertSlides.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void initServos() {
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

    public void holonomicDrive() {
        double vertical = -gamepad1.right_stick_y * slowedDownMulti;
        double horizontal = gamepad1.right_stick_x * slowedDownMulti;
        double pivot = gamepad1.left_stick_x * turnMulti;

        if (specimenMode){
            slowedDownMulti = 0.6;
        }
        else {
            slowedDownMulti = 0.8;
        }

        LF.setPower(pivot + vertical + horizontal);
        LB.setPower(pivot + vertical - horizontal);
        RF.setPower(-pivot + vertical - horizontal);
        RB.setPower(-pivot + vertical + horizontal);
    }

    public enum HorizontalSlidesState {
        IDLE,       // State when the process is not active
        START,      // Initial state
        WAIT_SLIDE, // Waiting for slides to reach position
        DONE        // Completed state
    }

    private HorizontalSlidesState slidesState = HorizontalSlidesState.IDLE;
    private ElapsedTime slidesTimer = new ElapsedTime();

    public void startHorizontalSlidesOut() {
        slidesState = HorizontalSlidesState.START;
    }

    public void handleHorizontalSlidesOut() {
        switch (slidesState) {
            case IDLE:
                // Do nothing when idle
                break;

            case START:
                // Initialize movement
                intakeClaw(intakeClawOpen);
                depositTransfer.setPosition(depositTransferIn);
                intakeTransfer.setPosition(0.75);
                intakeClawTilt.setPosition(0.05);
                horizSlides.setTargetPosition(1240);
                horizSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                horizSlides.setPower(1);
                slidesTimer.reset(); // Reset the timer for non-blocking delay
                slidesState = HorizontalSlidesState.WAIT_SLIDE;
                break;

            case WAIT_SLIDE:
                // Wait for the slide to finish moving or timeout
                if (!horizSlides.isBusy() || slidesTimer.milliseconds() > 700) {
                    intakeTransfer.setPosition(intakeTransferOut - 0.05);
                    slidesState = HorizontalSlidesState.DONE;
                }
                break;

            case DONE:
                // Task complete, reset state to idle
                slidesState = HorizontalSlidesState.IDLE;
                break;
        }
    }



    public void horizontalSlidesIn() {
        intakeTransfer.setPosition(0.75);
        intakeClawTilt.setPosition(0.05);
        intakeClawWrist.setPosition(intakeWristVert);
        horizSlides.setTargetPosition(0);
        horizSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizSlides.setPower(1);
    }



    //Pickup from Middle

    public enum PickupState {
        IDLE,       // State when the process is not active
        START,      // Initial state
        WAIT_OPEN,  // Waiting after opening the claw
        WAIT_CLOSE, // Waiting after closing the claw
        DONE        // Completed state
    }

    private PickupState pickupState = PickupState.IDLE;
    private ElapsedTime pickupTimer = new ElapsedTime();


    public void startPickupSample() {
        pickupState = PickupState.START;
    }

    public void handlePickupSample() {
        switch (pickupState) {
            case IDLE:
                break;

            case START:
                intakeClaw(intakeClawOpen);
                intakeTransfer.setPosition(intakeTransferOut);
                intakeClawTilt.setPosition(0.05);
                pickupTimer.reset();
                pickupState = PickupState.WAIT_OPEN;
                break;

            case WAIT_OPEN:
                if (pickupTimer.milliseconds() > 500) {
                    intakeClaw(intakeClawClose);
                    pickupTimer.reset();
                    pickupState = PickupState.WAIT_CLOSE;
                }
                break;

            case WAIT_CLOSE:
                if (pickupTimer.milliseconds() > 500) {
                    intakeTransfer.setPosition(intakeTransferOut-0.1);
                    pickupState = PickupState.DONE;
                }
                break;

            case DONE:
                pickupState = PickupState.IDLE;
                break;
        }
    }


    // Define the states for the transfer process
    public enum TransferState {
        IDLE,                 // State when the process is not active
        START,                  // Initial state
        DEP_SLIDES,
        WAIT_SLIDES,          // Wait for slides to finish moving
        WAIT_CLOSE_CLAW,      // Wait after closing the deposit claw
        WAIT_OPEN_CLAW,       // Wait after opening the intake claw
        DONE                  // Completed state
    }

    private TransferState transferState = TransferState.IDLE;
    private ElapsedTime transferTimer = new ElapsedTime();

    // Method to start the transfer process
    public void startTransfer() {
        transferState = TransferState.START;
    }

    // Method to handle the transfer state machine
    public void handleTransfer() {
        switch (transferState) {
            case IDLE:
                // Do nothing when idle
                break;

            case START:
                intakeTransfer.setPosition(intakeTransferIn);
                intakeClawTilt.setPosition(1);
                intakeClawWrist.setPosition(intakeWristVert);

                depositTransfer.setPosition(depositTransferIn);
                depositClaw(depositClawOpen);

                vertSlides.setTargetPosition(0);
                vertSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                vertSlides.setPower(1);

                horizSlides.setTargetPosition(290);
                horizSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                horizSlides.setPower(1);
                transferTimer.reset();
                transferState = TransferState.DEP_SLIDES;
                break;

            case DEP_SLIDES:
                if(transferTimer.milliseconds() > 700){

                    horizSlides.setTargetPosition(110);
                    horizSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    horizSlides.setPower(1);
                    transferTimer.reset();
                    transferState = TransferState.WAIT_SLIDES;
                }
                break;

            case WAIT_SLIDES:
                // Wait for slides to reach the target position or time out
                if (horizSlides.getCurrentPosition() <= 110 || transferTimer.milliseconds() > 550) {
                    depositClaw(depositClawClose);
                    transferTimer.reset();
                    transferState = TransferState.WAIT_CLOSE_CLAW;
                }
                break;

            case WAIT_CLOSE_CLAW:
                if (transferTimer.milliseconds() > 350) {
                    intakeClaw(intakeClawOpen);
                    transferTimer.reset();
                    transferState = TransferState.WAIT_OPEN_CLAW;
                }
                break;

            case WAIT_OPEN_CLAW:
                if (transferTimer.milliseconds() > 250) {
                    depositTransfer.setPosition(0.5);
                    intakeTransfer.setPosition(intakeTransferIn+0.1);
                    transferState = TransferState.DONE;
                }
                break;

            case DONE:
                // Process complete
                transferState = TransferState.IDLE; // Reset to idle
                break;
        }
    }



    public void highRung() {
        depositTransfer.setPosition(0.5);
        depositClaw(depositClawClose);

        vertSlides.setTargetPosition(highRungHeight);
        vertSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vertSlides.setPower(1);
    }

    public enum HighBasketState {
        IDLE,         // State when the process is not active
        START,        // Initial state
        WAIT_FOR_SLIDE, // Waiting for the slides to reach position
        DONE          // Completed state
    }

    private HighBasketState basketState = HighBasketState.IDLE;
    private ElapsedTime basketTimer = new ElapsedTime(); // Timer for optional timeout

    public void startHighBasket() {
        basketState = HighBasketState.START;
    }

    public void handleHighBasket() {
        switch (basketState) {
            case IDLE:
                // Do nothing when idle
                break;

            case START:
                // Start vertical slides movement
                vertSlides.setTargetPosition(highBasketHeight); // Bucket deposit height
                vertSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                vertSlides.setPower(1);
                basketState = HighBasketState.WAIT_FOR_SLIDE;
                break;

            case WAIT_FOR_SLIDE:
                // Wait until the slides reach the target position or exceed 2500
                if (vertSlides.getCurrentPosition() > 2500 || !vertSlides.isBusy()) {
                    depositTransfer.setPosition(0.7);
                    basketState = HighBasketState.DONE;
                }
                break;

            case DONE:
                // Task complete, reset state to idle
                basketState = HighBasketState.IDLE;
                break;
        }
    }



    //Slam the Specimen
    // Define the states for the slamSpecimen process
    public enum SlamState {
        IDLE,
        START,
        OPEN_CLAW,
        DONE
    }

    private SlamState slamState = SlamState.IDLE;
    private ElapsedTime slamTimer = new ElapsedTime();

    // Method to start the slamSpecimen process
    public void startSlamSpecimen() {
        slamState = SlamState.START;
    }

    // Method to handle the slamSpecimen state machine
    public void handleSlamSpecimen() {
        switch (slamState) {
            case IDLE:
                // Do nothing when idle
                break;

            case START:
                depositTransfer.setPosition(slamSpeciPos);
                slamTimer.reset();
                slamState = SlamState.OPEN_CLAW;
                break;

            case OPEN_CLAW:
                if (slamTimer.milliseconds() > 500) {
                    depositClaw(depositClawOpen);
                    depositTransfer.setPosition(0.7);
                    slamState = SlamState.DONE;
                }
                break;

            case DONE:
                // Process complete
                slamState = SlamState.IDLE; // Reset to idle
                break;
        }
    }

    public void specimenPickup() {
        depositClaw(depositClawOpen);
        depositTransfer.setPosition(0.975);
    }

    public void homePosition() {
        depositTransfer.setPosition(0.5);
        vertSlides.setTargetPosition(0); // Home position
        vertSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vertSlides.setPower(1);
    }

    public void intakeClaw(double pos) {
        intakeClawGrabLeft.setPosition(pos);
        intakeClawGrabRight.setPosition(pos);
    }

    public void depositClaw(double pos) {
        depositClawGrabRight.setPosition(pos);
        depositClawGrabLeft.setPosition(pos);
    }

}
