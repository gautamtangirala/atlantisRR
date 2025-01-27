package org.firstinspires.ftc.teamcode.opmodes.auto.future;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.auto.atlantisAutoEssentials;
import org.firstinspires.ftc.teamcode.rrfiles.PinpointDrive;


@Autonomous (preselectTeleOp = "atlantisTele")
public class fiveSample extends atlantisAutoEssentials {


    @Override
    public void runOpMode() {


        initDrive();
        initSubsystems();

        double fieldOffset = 0; //Change based on ridges of field
        Pose2d startPose = new Pose2d(-40, -62.625 + fieldOffset, Math.toRadians(270));
        PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);

        double dropX = -57, dropY = -56, dropAngle = Math.toRadians(45);

        Vector2d dropVector = new Vector2d(dropX, dropY);
        Pose2d dropPose = new Pose2d(dropX, dropY, dropAngle);






        Action dropPreload = drive.actionBuilder(startPose)
                .afterTime(0, moveSlideTop())

                .strafeToLinearHeading(dropVector, dropAngle)
                .waitSeconds(0.2)
                .stopAndAdd(depositTransferAction(0.7))
                .waitSeconds(0.4)
                .stopAndAdd(openDeposit())
                .waitSeconds(0.25)
                .stopAndAdd(depositTransferAction(0.5))
                .build();

        Action block2 = drive.actionBuilder(dropPose)
                .afterTime(0.1,moveSlideBottom())
                .afterTime(0, horizSlideOut(false))
                .strafeToLinearHeading(new Vector2d(-49.25, -50.5), Math.toRadians(90))
                .waitSeconds(0.5)

                //pickup block
                .stopAndAdd(clawPickup())
                .waitSeconds(0.7)
                .stopAndAdd(closeIntake())

                //transfer
                .waitSeconds(0.5)
                .stopAndAdd(transfer(1))
                .waitSeconds(0.5)
                .stopAndAdd(transfer(2))
                .waitSeconds(0.5)
                .stopAndAdd(closeDeposit())
                .waitSeconds(0.1)
                .stopAndAdd(openIntake())
                .waitSeconds(0.25)
                .stopAndAdd(transfer(3))

                //drop block
                .afterTime(0, moveSlideTop())
                .afterTime(1, horizSlideOut(false))
                .strafeToLinearHeading(dropVector, dropAngle)
                .waitSeconds(1)
                .stopAndAdd(depositTransferAction(0.7))
                .waitSeconds(0.25)
                .stopAndAdd(openDeposit())
                .waitSeconds(0.25)
                .stopAndAdd(depositTransferAction(0.5))
                .build();

        Action block3 = drive.actionBuilder(dropPose)
                .afterTime(0.1,moveSlideBottom())
                .afterTime(0, horizSlideOut(false))
                .strafeToLinearHeading(new Vector2d(-58.3, -51), Math.toRadians(90))
                .waitSeconds(0.5)

                //pickup block
                .stopAndAdd(clawPickup())
                .waitSeconds(0.7)
                .stopAndAdd(closeIntake())

                //transfer
                .waitSeconds(0.5)
                .stopAndAdd(transfer(1))
                .waitSeconds(0.5)
                .stopAndAdd(transfer(2))
                .waitSeconds(0.5)
                .stopAndAdd(closeDeposit())
                .waitSeconds(0.1)
                .stopAndAdd(openIntake())
                .waitSeconds(0.5)
                .stopAndAdd(transfer(3))

                //drop block
                .afterTime(0, moveSlideTop())
                .afterTime(1, horizSlideOut(false))
                .strafeToLinearHeading(dropVector, dropAngle)
                .waitSeconds(1)
                .stopAndAdd(depositTransferAction(0.7))
                .waitSeconds(0.25)
                .stopAndAdd(openDeposit())
                .waitSeconds(0.5)
                .stopAndAdd(depositTransferAction(0.5))
                .build();

        Action block4 = drive.actionBuilder(dropPose)
                .afterTime(0.1,moveSlideBottom())
                .afterTime(0, horizSlideOut(false))
                .afterTime(0, moveWrist(0))
                .strafeToLinearHeading(new Vector2d(-45.25, -26), Math.toRadians(180))
                .waitSeconds(0.5)

                //pickup block
                .stopAndAdd(clawPickup())
                .waitSeconds(0.7)
                .stopAndAdd(closeIntake())

                //transfer
                .waitSeconds(0.5)
                .stopAndAdd(transfer(1))
                .waitSeconds(0.5)
                .stopAndAdd(transfer(2))
                .waitSeconds(0.5)
                .stopAndAdd(closeDeposit())
                .waitSeconds(0.1)
                .stopAndAdd(openIntake())
                .waitSeconds(0.5)
                .stopAndAdd(transfer(3))

                //drop block
                .afterTime(0, moveSlideTop())
                .strafeToLinearHeading(dropVector, dropAngle)
                .waitSeconds(1)
                .stopAndAdd(depositTransferAction(0.7))
                .waitSeconds(0.5)
                .stopAndAdd(openDeposit())
                .waitSeconds(0.5)
                .stopAndAdd(depositTransferAction(0.5))
                .build();


        Action end = drive.actionBuilder(dropPose)
                .afterTime(0.5,moveSlidePark())
                .splineToLinearHeading(new Pose2d(-25, -5, Math.toRadians(180)), Math.toRadians(0))
                .stopAndAdd(depositTransferAction(0.7))
                .waitSeconds(0.1)
                .build();


        depositGrab(depositClawClose);
        depositTransfer.setPosition(0.5);
        intakeTransfer.setPosition(intakeTransferIn);




        telemetry.update();

        double xMin = -1, yMin = -1;
        double sample5X = xMin, sample5Y = yMin;
        boolean sample5Vert = true;

// Variables to track last button press times
        long lastPressTime = 0;
        long debounceTime = 500;

        while (opModeInInit()) {

            double bigIncrement = 2.5;
            double smallIncrement = 0.5;

            // Current time
            long currentTime = System.currentTimeMillis();

            // Adjust Y with big increments (debounced)
            if (currentTime - lastPressTime >= debounceTime) {
                if (gamepad1.dpad_up) {
                    sample5Y += bigIncrement;
                    lastPressTime = currentTime;
                } else if (gamepad1.dpad_down) {
                    sample5Y -= bigIncrement;
                    lastPressTime = currentTime;
                }

                // Adjust X with big increments
                if (gamepad1.dpad_right) {
                    sample5X += bigIncrement;
                    lastPressTime = currentTime;
                } else if (gamepad1.dpad_left) {
                    sample5X -= bigIncrement;
                    lastPressTime = currentTime;
                }

                // Adjust Y with small increments
                if (gamepad1.y) {
                    sample5Y += smallIncrement;
                    lastPressTime = currentTime;
                } else if (gamepad1.a) {
                    sample5Y -= smallIncrement;
                    lastPressTime = currentTime;
                }

                // Adjust X with small increments
                if (gamepad1.b) {
                    sample5X += smallIncrement;
                    lastPressTime = currentTime;
                } else if (gamepad1.x) {
                    sample5X -= smallIncrement;
                    lastPressTime = currentTime;
                }

                // Set orientation
                if (gamepad1.right_bumper) {
                    sample5Vert = true;
                    lastPressTime = currentTime;
                } else if (gamepad2.left_bumper) {
                    sample5Vert = false;
                    lastPressTime = currentTime;
                }
            }

            // Enforce limits on X and Y
            sample5X = Math.min(xMin, Math.max(-10, sample5X)); // Limit X to [0, -10]
            sample5Y = Math.min(yMin, Math.max(-10, sample5Y)); // Limit Y to [0, -10]

            // Telemetry data

            telemetry.addLine("5 SAMPLE AUTO: 80 POINTS");
            telemetry.addLine("ALIGN ROBOT SIDE WITH NET ZONE");
            telemetry.addLine(" ");
            telemetry.addData("5th Sample X", sample5X);
            telemetry.addData("5th Sample Y", sample5Y);
            telemetry.addData("5th Sample Orientation", sample5Vert ? "Vertical" : "Horizontal");
            telemetry.update();
        }



        waitForStart();

        Action block5 = drive.actionBuilder(dropPose)
                .afterTime(0.1,moveSlideBottom())
                .splineToLinearHeading(new Pose2d(-30, -5, Math.toRadians(0)), Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(sample5X - 20, sample5Y))
                .afterTime(0, horizSlideOut(true))
                .afterTime(0, moveWrist(sample5Vert ? 0.525: 0))
                .waitSeconds(0.5)

                //pickup block
                .stopAndAdd(clawPickup())
                .waitSeconds(0.7)
                .stopAndAdd(closeIntake())

                //transfer
                .waitSeconds(0.5)
                .stopAndAdd(transfer(1))
                .waitSeconds(0.5)
                .stopAndAdd(transfer(2))
                .waitSeconds(0.5)
                .stopAndAdd(closeDeposit())
                .waitSeconds(0.1)
                .stopAndAdd(openIntake())
                .waitSeconds(0.5)
                .stopAndAdd(transfer(3))

                //drop block
                .strafeToConstantHeading(new Vector2d(-40,sample5Y))
                .afterTime(0, moveSlideTop())
                .strafeToLinearHeading(dropVector, dropAngle)
                .waitSeconds(1)
                .stopAndAdd(depositTransferAction(0.7))
                .waitSeconds(0.5)
                .stopAndAdd(openDeposit())
                .waitSeconds(0.5)
                .stopAndAdd(depositTransferAction(0.5))
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        dropPreload,
                        block2,
                        block3,
                        block4,
                        block5,
                        end
                )
        );

    }


}
