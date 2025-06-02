package org.firstinspires.ftc.teamcode.opmodes.auto.current;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.auto.atlantisAutoEssentials;
import org.firstinspires.ftc.teamcode.rrfiles.PinpointDrive;


@Autonomous (name = "Six Sample Red", preselectTeleOp = "atlantisTele")
public class sixSampleVisionFullNN extends atlantisAutoEssentials {


    @Override
    public void runOpMode() {


        initDrive();
        initSubsystems();
        initLimelight();
        limelight.pipelineSwitch(2);

        double fieldOffset = 0.5; //Change based on ridges of field
        String color = "red";
        Pose2d startPose = new Pose2d(-40, -62.625 + fieldOffset, Math.toRadians(270));
        PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);

        double dropX = -57, dropY = -57, dropAngle = Math.toRadians(45);

        Vector2d dropVector = new Vector2d(dropX, dropY);
        Pose2d dropPose = new Pose2d(dropX, dropY, dropAngle);






        Action dropPreload = drive.actionBuilder(startPose)
                .afterTime(0, moveSlideHighBasket())
                .afterTime(0, openIntake())

                .strafeToLinearHeading(dropVector, dropAngle)
                .stopAndAdd( new SequentialAction(moveSlideHighBasket(), depositTransferAction(0.7), new SleepAction(0.15), openDeposit(), new SleepAction(0.2), depositTransferAction(0.5)))
                .build();

        Action block2 = drive.actionBuilder(dropPose)
                .afterTime(0.1,moveSlideBottom())
                .afterTime(0.5, horizSlideOut(false))
                .strafeToLinearHeading(new Vector2d(-49.25, -51), Math.toRadians(90))
                .stopAndAdd(new SequentialAction(intakeTransferAction(0.9),new SleepAction(0.1)))
                        .stopAndAdd(clawPickup())
                        .stopAndAdd(new SequentialAction( new SleepAction(0.5), closeIntake(), new SleepAction(0.1)))

                //transfer
                .afterTime(0, transfer(1))
                .afterTime(0.5, closeDeposit())
                .afterTime(0.6, openIntake())
                .afterTime(0.8, transfer(3))

                //drop block
                .afterTime(0.8, new SequentialAction(moveSlideHighBasket(), depositTransferAction(0.7), new SleepAction(0.15), openDeposit(), new SleepAction(0.2), depositTransferAction(0.5)))
                .strafeToLinearHeading(dropVector, dropAngle)
                .build();

        Action block3 = drive.actionBuilder(dropPose)
                .afterTime(0.1,moveSlideBottom())
                .afterTime(0.5, horizSlideOut(false))
                .strafeToLinearHeading(new Vector2d(-59.25, -51), Math.toRadians(90))
                .stopAndAdd(new SequentialAction(horizSlideOut(false), intakeTransferAction(0.9),new SleepAction(0.1)))
                .stopAndAdd(clawPickup())
                .stopAndAdd(new SequentialAction( clawPickup(), new SleepAction(0.5), closeIntake(), new SleepAction(0.1)))

                //transfer
                .afterTime(0, transfer(1))
                .afterTime(0.5, closeDeposit())
                .afterTime(0.6, openIntake())
                .afterTime(0.8, transfer(3))

                //drop block
                .afterTime(0.8, new SequentialAction(moveSlideHighBasket(), depositTransferAction(0.7), new SleepAction(0.15), openDeposit(), new SleepAction(0.2), depositTransferAction(0.5)))
                .strafeToLinearHeading(dropVector, dropAngle)
                .build();

        Action block4 = drive.actionBuilder(dropPose)
                .afterTime(0.1,moveSlideBottom())
                .afterTime(1, new ParallelAction(horizSlideOut(false), moveWrist(0)))
                .strafeToLinearHeading(new Vector2d(-44.25, -27.5), Math.toRadians(180))
                .stopAndAdd( new SequentialAction(intakeTransferAction(0.9), new SleepAction(0.1), clawPickup(), new SleepAction(0.5), closeIntake(), new SleepAction(0.1)))

                //transfer
                .afterTime(0, transfer(1))
                .afterTime(0.5, closeDeposit())
                .afterTime(0.6, openIntake())
                .afterTime(0.8, transfer(3))

                //drop block
                .afterTime(0.8, new SequentialAction(moveSlideHighBasket(), depositTransferAction(0.7), new SleepAction(0.15), openDeposit(), new SleepAction(0.2), depositTransferAction(0.5)))
                .strafeToLinearHeading(dropVector, dropAngle)
                .build();


        Action end = drive.actionBuilder(dropPose)
                .afterTime(0.1,moveSlideBottom())
                .afterTime(1, depositTransferAction(0.5))
                .splineToLinearHeading(new Pose2d(-25, -10, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.1)
                .build();


        depositGrab(depositClawClose);
        depositTransfer.setPosition(0.4);
        intakeTransfer.setPosition(intakeTransferIn);
        intakeClawTilt.setPosition(0.85);
        intakeClawWrist.setPosition(intakeWristVert);
        limelight.setPollRateHz(100);
        limelight.start();




        double xMin = -3, yMin = -10;
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

            telemetry.addLine("     **RED SIDE**     ");
            telemetry.addLine("6 SAMPLE AUTO: 96 POINTS");
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

        Action block5Sub = drive.actionBuilder(dropPose)
                .afterTime(0.1,moveSlideBottom())
                .afterTime(0, moveWrist(sample5Vert ? 0.525: 0))
                .splineToLinearHeading(new Pose2d(sample5X -25.5, sample5Y, Math.toRadians(0)), Math.toRadians(0), new TranslationalVelConstraint(90))
                 .build();



        Action block6Sub = drive.actionBuilder(dropPose)
                    .afterTime(0.1,moveSlideBottom())
                    .afterTime(0, moveWrist(sample5Vert ? 0.525: 0))
                    .splineToLinearHeading(new Pose2d(sample5X -25.5, sample5Y, Math.toRadians(0)), Math.toRadians(0), new TranslationalVelConstraint(90))
                .build();


        Actions.runBlocking(
                new ParallelAction( updatePidAction(),
                new SequentialAction(
                        dropPreload,
                        block2,
                        endPID()
                ))
        );


        Actions.runBlocking(
                new ParallelAction( updatePidAction(),
                        new SequentialAction(
                                block3,
                                block4,
                                block5Sub,
                                new SleepAction(0.15),
                                updateLimelightNeural(color),
                                endPID()
                        ))
        );
       

        Pose2d currPose = drive.pose;
        TrajectoryActionBuilder block5PickDrop = drive.actionBuilder(currPose)
                .strafeToConstantHeading(new Vector2d(currPose.position.x , currPose.position.y - inchesToBlock))
                .stopAndAdd(new SequentialAction(new ParallelAction(horizSlideOut(true, ticksToBlock), moveWrist(clawPosBlock)), new SleepAction(0.1), clawPickup(), new SleepAction(0.5), closeIntake(), new SleepAction(0.25)))

                //transfer
                .afterTime(0, transfer(1))
                .afterTime(0.5, closeDeposit())
                .afterTime(0.6, openIntake())
                .afterTime(0.8, new ParallelAction( transfer(3), new SequentialAction(moveSlideHighBasket(), depositTransferAction(0.7))))

                //drop block
                .splineToConstantHeading(new Vector2d(-45, sample5Y), Math.toRadians(270), new TranslationalVelConstraint(80))
                .splineToSplineHeading(dropPose, Math.toRadians(270), new TranslationalVelConstraint(80))
                .stopAndAdd(new SequentialAction(new SleepAction(0.1), openDeposit(), new SleepAction(0.25), depositTransferAction(0.5)))
                ;

        Actions.runBlocking(      new ParallelAction( updatePidAction(),
                new SequentialAction(
                        block5PickDrop.build(),
                        block6Sub,
                        new SleepAction(0.15),
                        updateLimelightNeural(color),
                        stopLimelight(),
                        endPID()
                ))
                );

        TrajectoryActionBuilder block6PickDrop = drive.actionBuilder(currPose)
                .strafeToConstantHeading(new Vector2d(currPose.position.x, currPose.position.y - inchesToBlock))
                .stopAndAdd(new SequentialAction(new ParallelAction(horizSlideOut(true, ticksToBlock), moveWrist(clawPosBlock)), new SleepAction(0.1), clawPickup(), new SleepAction(0.5), closeIntake(), new SleepAction(0.25)))

                //transfer
                .afterTime(0, transfer(1))
                .afterTime(0.5, closeDeposit())
                .afterTime(0.6, openIntake())
                .afterTime(0.8, new ParallelAction( transfer(3), new SequentialAction(moveSlideHighBasket(), depositTransferAction(0.7))))


                //drop block
                .splineToConstantHeading(new Vector2d(-45, sample5Y), Math.toRadians(270), new TranslationalVelConstraint(80))
                .splineToSplineHeading(dropPose, Math.toRadians(270), new TranslationalVelConstraint(80))
                .stopAndAdd(new SequentialAction(new SleepAction(0.1), openDeposit(), new SleepAction(0.25), depositTransferAction(0.5)))
                ;

        Actions.runBlocking(      new ParallelAction( updatePidAction(),
                new SequentialAction(
                        block6PickDrop.build(),
                        end,
                        endPID()
                ))
        );




    }

}