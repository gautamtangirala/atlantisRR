package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Disabled
@TeleOp(name = "solo teleop")
public class atlantisTeleSolo extends LinearOpMode {
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
    double depositClawClose = 0.45;
    double depositClawOpen = 0.1;

    double depositTransferIn = 0.1725;
    double depositTransferOut = 1;
    double slamSpeciPos = 1;

    double intakeClawClose = 0.45;  
    double intakeClawOpen = 0.1;

    double intakeTransferOut = 1;
    double intakeTransferIn = 0.4;

    int highRungHeight = 525;
    int highBasketHeight = 970;


    int horizTransferPos = 34;
    int horizOutPos = 470;

    double intakeWristVert = 0.525;
    double intakeWristHoriz = 0;

    double turnMulti = 0.75;
    double slowedDownMulti = 0.8;

    boolean specimenMode = false;
    boolean sampleMode = true;







    @Override
    public void runOpMode() {
        initMotors();
        initServos();

        waitForStart();

        horizSetPoint = horizSlides.getCurrentPosition();
        vertSetPoint = vertSlides.getCurrentPosition();

        while (opModeIsActive()) {
            updatePID();
            // Gamepad 1: Holonomic drive
            holonomicDrive();

            if (specimenMode) {
                // Specimen Mode Controls
                if (gamepad1.dpad_up) {
                    highRung();
                } else if (gamepad1.dpad_right) {
                    if (slamState == SlamState.IDLE) {
                        startSlamSpecimen();
                    }
                } else if (gamepad1.dpad_left) {
                    specimenPickup();
                } else if (gamepad1.dpad_down) {
                    homePosition();
                } else if (gamepad1.left_bumper) {
                    startHorizontalSlidesOut();
                } else if (gamepad1.right_bumper) {
                    horizontalSlidesIn();
                }

                // Common Controls
                if (gamepad1.b) {
                    depositClaw(depositClawOpen);
                } else if (gamepad1.x) {
                    depositClaw(depositClawClose);
                } else if (gamepad1.a && pickupState == PickupState.IDLE ) {
                    startPickupSample();
                } else if (gamepad1.y) {
                    startTransfer();
                }
            } else if (sampleMode) {
                // Sample Mode Controls
                if (gamepad1.dpad_up && transferState == TransferState.IDLE) {
                    startHighBasket();
                } else if (gamepad1.right_bumper && transferState == TransferState.IDLE && pickupState == PickupState.IDLE) {
                    startTransfer();
                } else if (gamepad1.dpad_down) {
                    homePosition();
                } else if (gamepad1.left_bumper) {
                    startHorizontalSlidesOut();
                } else if (gamepad1.dpad_right) {
                    horizontalSlidesIn();
                } else if(gamepad1.dpad_left){

                }

                // Common Controls
                if (gamepad1.b) {
                    depositClaw(depositClawOpen);
                } else if (gamepad1.x) {
                    depositClaw(depositClawClose);
                } else if (gamepad1.a && pickupState == PickupState.IDLE && transferState == TransferState.IDLE) {
                    startPickupSample();
                } else if (gamepad1.y) {
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

            if (gamepad1.left_trigger != 0) {
                intakeClawWrist.setPosition(intakeWristHoriz);
            } else {
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
        horizSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        vertSlides = hardwareMap.get(DcMotorEx.class, "vertSlide");
        vertSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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

                depositTransfer.setPosition(depositTransferIn);
                intakeTransfer.setPosition(0.75);
                intakeClawTilt.setPosition(0.05);
                horizSetPoint = horizOutPos;
                slidesTimer.reset(); // Reset the timer for non-blocking delay
                slidesState = HorizontalSlidesState.WAIT_SLIDE;
                break;

            case WAIT_SLIDE:
                // Wait for the slide to finish moving or timeout
                if (horizSlides.getCurrentPosition() >= 0.6*horizOutPos || slidesTimer.milliseconds() > 700) {
                    intakeTransfer.setPosition(intakeTransferOut - 0.05);
                    slidesState = HorizontalSlidesState.DONE;
                }
                break;

            case DONE:
                if (horizSlides.getCurrentPosition() >= 0.9 * horizOutPos) {
                    intakeClaw(intakeClawOpen);
                    slidesState = HorizontalSlidesState.IDLE;
                }
                break;
        }
    }



    public void horizontalSlidesIn() {

        intakeTransfer.setPosition(0.75);
        intakeClawTilt.setPosition(0.05);
        intakeClawWrist.setPosition(intakeWristVert);
        horizSetPoint = 0;
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
                intakeClawTilt.setPosition(0.075);
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
                if (pickupTimer.milliseconds() > 250) {
                    intakeTransfer.setPosition(intakeTransferOut-0.1);
                    intakeClawTilt.setPosition(0);
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
                intakeClawTilt.setPosition(0.8);
                intakeClawWrist.setPosition(intakeWristVert);

                depositTransfer.setPosition(depositTransferIn);
                depositClaw(depositClawOpen);

               vertSetPoint = 0;

                horizSetPoint = horizTransferPos;
                transferTimer.reset();
                transferState = TransferState.DEP_SLIDES;
                break;

            case DEP_SLIDES:
                if(transferTimer.milliseconds() > 300){

                    horizSetPoint = horizTransferPos;
                    transferTimer.reset();
                    transferState = TransferState.WAIT_SLIDES;
                }
                break;

            case WAIT_SLIDES:
                // Wait for slides to reach the target position or time out
                if (transferTimer.milliseconds() > 500) {
                    depositClaw(depositClawClose);
                    transferTimer.reset();
                    transferState = TransferState.WAIT_CLOSE_CLAW;
                }
                break;

            case WAIT_CLOSE_CLAW:
                if (transferTimer.milliseconds() > 100) {
                    intakeClaw(intakeClawOpen);

                    transferTimer.reset();
                    transferState = TransferState.WAIT_OPEN_CLAW;
                }
                break;

            case WAIT_OPEN_CLAW:
                if (transferTimer.milliseconds() > 150) {
                    intakeTransfer.setPosition(intakeTransferIn+0.3);
                    depositTransfer.setPosition(0.5);
                    transferState = TransferState.DONE;
                }
                break;

            case DONE:
                // Process complete
                transferState = TransferState.IDLE;
                intakeTransfer.setPosition(intakeTransferIn);

                break;
        }
    }



    public void highRung() {
        depositClaw(depositClawClose);
        vertSetPoint = highRungHeight;
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
                vertSetPoint = highBasketHeight;
                basketState = HighBasketState.WAIT_FOR_SLIDE;
                break;

            case WAIT_FOR_SLIDE:
                // Wait until the slides reach the target position or exceed 2500
                if (vertSlides.getCurrentPosition() > highBasketHeight * 0.8) {
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
    private double offset =  142;

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
                vertSetPoint = highRungHeight - offset;
                slamTimer.reset();
                slamState = SlamState.OPEN_CLAW;
                break;

            case OPEN_CLAW:
                if (Math.abs(vertSlides.getCurrentPosition() - offset) > 5 || slamTimer.seconds() > 1) {
                    slamTimer.reset();
                    slamState = SlamState.DONE;
                }
                break;

            case DONE:
                // Process complete
                if(slamTimer.milliseconds() > 100){
                    depositClaw(depositClawOpen);
                    slamState = SlamState.IDLE;
                }
                 // Reset to idle
                break;
        }
    }

    public void specimenPickup() {
        intakeClawTilt.setPosition(0.8);
        intakeTransfer.setPosition(intakeTransferIn);
        depositClaw(depositClawOpen);
        depositTransfer.setPosition(0.975);
        vertSetPoint = 0;
    }

    public void homePosition() {
        depositTransfer.setPosition(0.5);
        intakeClawTilt.setPosition(0.8);
        intakeTransfer.setPosition(intakeTransferIn);
        vertSetPoint = 0;
        horizSetPoint = 0;
    }

    public void intakeClaw(double pos) {
        intakeClawGrabLeft.setPosition(pos);
        intakeClawGrabRight.setPosition(pos);
    }

    public void depositClaw(double pos) {
        depositClawGrabRight.setPosition(pos);
        depositClawGrabLeft.setPosition(pos);
    }


    public static double vertP = 0.035;
    public static double vertI = 0.00002;
    public static double vertD = 0.0003;
    public static double vertF = 0.00018;
    private static final PIDFController vertSlidePIDF = new PIDFController(vertP, vertI, vertD, vertF);
    public static double vertSetPoint;
    public static double vertMaxPowerConstant = 1.0;
    int vertMotorPosition;


    public static double horizP = 0.025;
    public static double horizI = 0;
    public static double horizD = 0.0002;
    public static double horizF = 0.0001;
    private static final PIDFController horizSlidePIDF = new PIDFController(horizP, horizI, horizD, horizF);
    public static double horizSetPoint;
    public static double horizMaxPowerConstant = 0.8;
    int horizMotorPosition;

    public void updatePID(){
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
    }



}
