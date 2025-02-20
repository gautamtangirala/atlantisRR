package org.firstinspires.ftc.teamcode.opmodes.auto.current;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.auto.atlantisAutoEssentials;
import org.firstinspires.ftc.teamcode.rrfiles.PinpointDrive;


@Autonomous (name = "Five Sample", preselectTeleOp = "atlantisTele")
public class fiveSample extends atlantisAutoEssentials {


    @Override
    public void runOpMode() {


        initDrive();
        initSubsystems();

        double fieldOffset = 0; //Change based on ridges of field
        Pose2d startPose = new Pose2d(-40, -62.625 + fieldOffset, Math.toRadians(270));
        PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);

        double dropX = -57, dropY = -57, dropAngle = Math.toRadians(45);

        Vector2d dropVector = new Vector2d(dropX, dropY);
        Pose2d dropPose = new Pose2d(dropX, dropY, dropAngle);






        Action dropPreload = drive.actionBuilder(startPose)
                .afterTime(0, moveSlideHighBasket())
                .afterTime(0, openIntake())

                .strafeToLinearHeading(dropVector, dropAngle)
                .stopAndAdd( new SequentialAction(moveSlideHighBasket(), depositTransferAction(0.7), new SleepAction(0.15), openDeposit(), new SleepAction(0.35), depositTransferAction(0.5)))
                .build();

        Action block2 = drive.actionBuilder(dropPose)
                .afterTime(0.1,moveSlideBottom())
                .strafeToLinearHeading(new Vector2d(-49.25, -51), Math.toRadians(90))
                .stopAndAdd(new SequentialAction(horizSlideOut(false), new SleepAction(0.1), clawPickup(), new SleepAction(0.5), closeIntake(), new SleepAction(0.1)))

                //transfer
                .afterTime(0, transfer(1))
                .afterTime(0.5, transfer(2))
                .afterTime(1, closeDeposit())
                .afterTime(1.1, openIntake())
                .afterTime(1.35, transfer(3))



                //drop block
                .afterTime(1.35, new SequentialAction(moveSlideHighBasket(), depositTransferAction(0.7), new SleepAction(0.15), openDeposit(), new SleepAction(0.35), depositTransferAction(0.5)))
                .strafeToLinearHeading(dropVector, dropAngle)
                .build();

        Action block3 = drive.actionBuilder(dropPose)
                .afterTime(0.1,moveSlideBottom())
                .strafeToLinearHeading(new Vector2d(-59.25, -51), Math.toRadians(90))
                .stopAndAdd(new SequentialAction(horizSlideOut(false), new SleepAction(0.1), clawPickup(), new SleepAction(0.5), closeIntake(), new SleepAction(0.1)))

                //transfer
                .afterTime(0, transfer(1))
                .afterTime(0.5, transfer(2))
                .afterTime(1, closeDeposit())
                .afterTime(1.1, openIntake())
                .afterTime(1.35, transfer(3))

                //drop block
                .afterTime(1.35, new SequentialAction(moveSlideHighBasket(), depositTransferAction(0.7), new SleepAction(0.15), openDeposit(), new SleepAction(0.35), depositTransferAction(0.5)))
                .strafeToLinearHeading(dropVector, dropAngle)
                .build();

        Action block4 = drive.actionBuilder(dropPose)
                .afterTime(0.1,moveSlideBottom())
                .strafeToLinearHeading(new Vector2d(-44.25, -28), Math.toRadians(180))
                .stopAndAdd( new SequentialAction( new ParallelAction(horizSlideOut(false), moveWrist(0)), intakeTransferAction(0.9), new SleepAction(0.1), clawPickup(), new SleepAction(0.5), closeIntake(), new SleepAction(0.1)))

                //transfer
                .afterTime(0, transfer(1))
                .afterTime(0.5, transfer(2))
                .afterTime(1, closeDeposit())
                .afterTime(1.1, openIntake())
                .afterTime(1.35, transfer(3))

                //drop block
                .afterTime(1.35, new SequentialAction(moveSlideHighBasket(), depositTransferAction(0.7), new SleepAction(0.15), openDeposit(), new SleepAction(0.35), depositTransferAction(0.5)))
                .strafeToLinearHeading(dropVector, dropAngle)
                .build();


        Action end = drive.actionBuilder(dropPose)
                .afterTime(0.5,moveSlidePark())
                .afterTime(1, depositTransferAction(0.7))
                .splineToLinearHeading(new Pose2d(-25, -10, Math.toRadians(180)), Math.toRadians(0))
                .waitSeconds(0.1)
                .build();


        depositGrab(depositClawClose);
        depositTransfer.setPosition(0.5);
        intakeTransfer.setPosition(intakeTransferIn);
        intakeClawTilt.setPosition(0.85);




        double xMin = 0, yMin = 0;
        double sample5X = xMin, sample5Y = yMin;
        boolean sample5Vert = true;

// Variables to track last button press times
        long lastPressTime = 0;
        long debounceTime = 300;

        while (opModeInInit()) {

            double bigIncrement = 1;
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
                } else if (gamepad1.left_bumper) {
                    sample5Vert = false;
                    lastPressTime = currentTime;
                }
            }

            // Enforce limits on X and Y
            sample5X = Math.min(xMin, Math.max(-12, sample5X));
            sample5Y = Math.min(yMin, Math.max(-15, sample5Y));

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

        sample5Y *= 1.2;
        sample5X *= 1.2;

        Action block5 = drive.actionBuilder(dropPose)
                .afterTime(0.1,moveSlideBottom())
                .afterTime(0, moveWrist(sample5Vert ? 0.525: 0))
                .splineToLinearHeading(new Pose2d(-32, sample5Y, Math.toRadians(0)), Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(sample5X - 25.5, sample5Y))
                .stopAndAdd( new SequentialAction( new ParallelAction(horizSlideOut(true), moveWrist(sample5Vert ? 0.525: 0)), new SleepAction(0.1), clawPickup(), new SleepAction(0.5), closeIntake(), new SleepAction(0.25)))

                //transfer
                .afterTime(0, transfer(1))
                .afterTime(0.5, transfer(2))
                .afterTime(1, closeDeposit())
                .afterTime(1.1, openIntake())
                .afterTime(1.35, new ParallelAction( transfer(3), new SequentialAction(moveSlideHighBasket(), depositTransferAction(0.7))))

                //drop block
                .strafeToConstantHeading(new Vector2d(-45, sample5Y))
                .strafeToLinearHeading(dropVector, dropAngle)
                .stopAndAdd(new SequentialAction(new SleepAction(0.1), openDeposit(), new SleepAction(0.25), depositTransferAction(0.5)))
                .build();



        Actions.runBlocking(
                new ParallelAction( updatePidAction(),
                new SequentialAction(
                        dropPreload,
                        block2,
                        block3,
                        block4,
                        block5,
                        end
                ))
        );

    }

}
