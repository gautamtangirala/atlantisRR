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


@Autonomous (name = "Six Sample Red Gamepad ", preselectTeleOp = "atlantisTele")
public class sixSampleVisionGamepad extends atlantisAutoEssentials {


    @Override
    public void runOpMode() {


        initDrive();
        initSubsystems();

        double fieldOffset = 0.5; //Change based on ridges of field
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





        // Manual input for sample 5 and 6
        double sample5X = 0, sample5Y = 0, sample6X = 0, sample6Y = 0;
        boolean sample5Vert = true, sample6Vert = true;

        long lastPressTime = 0, debounceTime = 300;
        while (opModeInInit()) {
            double bigIncrement = 1, smallIncrement = 0.5;
            long currentTime = System.currentTimeMillis();

            if (currentTime - lastPressTime >= debounceTime) {
                if (gamepad1.dpad_up) { sample5Y += bigIncrement; lastPressTime = currentTime; }
                if (gamepad1.dpad_down) { sample5Y -= bigIncrement; lastPressTime = currentTime; }
                if (gamepad1.dpad_right) { sample5X += bigIncrement; lastPressTime = currentTime; }
                if (gamepad1.dpad_left) { sample5X -= bigIncrement; lastPressTime = currentTime; }
                if (gamepad1.y) { sample5Y += smallIncrement; lastPressTime = currentTime; }
                if (gamepad1.a) { sample5Y -= smallIncrement; lastPressTime = currentTime; }
                if (gamepad1.b) { sample5X += smallIncrement; lastPressTime = currentTime; }
                if (gamepad1.x) { sample5X -= smallIncrement; lastPressTime = currentTime; }
                if (gamepad1.right_bumper) { sample5Vert = true; lastPressTime = currentTime; }
                if (gamepad1.left_bumper) { sample5Vert = false; lastPressTime = currentTime; }

                // Sample 6 input (using second gamepad for differentiation)
                if (gamepad2.dpad_up) { sample6Y += bigIncrement; lastPressTime = currentTime; }
                if (gamepad2.dpad_down) { sample6Y -= bigIncrement; lastPressTime = currentTime; }
                if (gamepad2.dpad_right) { sample6X += bigIncrement; lastPressTime = currentTime; }
                if (gamepad2.dpad_left) { sample6X -= bigIncrement; lastPressTime = currentTime; }
                if (gamepad2.y) { sample6Y += smallIncrement; lastPressTime = currentTime; }
                if (gamepad2.a) { sample6Y -= smallIncrement; lastPressTime = currentTime; }
                if (gamepad2.b) { sample6X += smallIncrement; lastPressTime = currentTime; }
                if (gamepad2.x) { sample6X -= smallIncrement; lastPressTime = currentTime; }
                if (gamepad2.right_bumper) { sample6Vert = true; lastPressTime = currentTime; }
                if (gamepad2.left_bumper) { sample6Vert = false; lastPressTime = currentTime; }
            }

            telemetry.addData("5th Sample X", sample5X);
            telemetry.addData("5th Sample Y", sample5Y);
            telemetry.addData("5th Sample Orientation", sample5Vert ? "Vertical" : "Horizontal");
            telemetry.addData("6th Sample X", sample6X);
            telemetry.addData("6th Sample Y", sample6Y);
            telemetry.addData("6th Sample Orientation", sample6Vert ? "Vertical" : "Horizontal");
            telemetry.update();
        }

        waitForStart();

       // sample5Y *= 1.2; sample5X *= 1.2;
      //  sample6Y *= 1.2; sample6X *= 1.2;

        Action block5 = drive.actionBuilder(dropPose)
                .afterTime(0.1, moveSlideBottom())
                .afterTime(0, moveWrist(sample5Vert ? 0.525 : 0))

                .splineToLinearHeading(new Pose2d(sample5X - 25.5, sample5Y, Math.toRadians(0)), Math.toRadians(0))
                .stopAndAdd(new SequentialAction(horizSlideOut(true), clawPickup(), new SleepAction(0.5), closeIntake(), new SleepAction(0.25)))
                .afterTime(0, transfer(1))
                .afterTime(0.5, transfer(2))
                .afterTime(1, closeDeposit())
                .afterTime(1.1, openIntake())
                .afterTime(1.35, new ParallelAction(transfer(3), new SequentialAction(moveSlideHighBasket(), depositTransferAction(0.7))))
                .strafeToLinearHeading(dropVector, dropAngle, new TranslationalVelConstraint(90))
                .stopAndAdd(new SequentialAction(new SleepAction(0.1), openDeposit(), new SleepAction(0.25), depositTransferAction(0.5)))
                .build();

        Action block6 = drive.actionBuilder(dropPose)
                .afterTime(0.1, moveSlideBottom())
                .afterTime(0, moveWrist(sample6Vert ? 0.525 : 0))
                .splineToLinearHeading(new Pose2d(sample6X - 25.5, sample6Y, Math.toRadians(0)), Math.toRadians(0))
                .stopAndAdd(new SequentialAction(horizSlideOut(true), clawPickup(), new SleepAction(0.5), closeIntake(), new SleepAction(0.25)))
                .afterTime(0, transfer(1))
                .afterTime(0.5, transfer(2))
                .afterTime(1, closeDeposit())
                .afterTime(1.1, openIntake())
                .afterTime(1.35, new ParallelAction(transfer(3), new SequentialAction(moveSlideHighBasket(), depositTransferAction(0.7))))
                .strafeToLinearHeading(dropVector, dropAngle, new TranslationalVelConstraint(90))
                .stopAndAdd(new SequentialAction(new SleepAction(0.1), openDeposit(), new SleepAction(0.25), depositTransferAction(0.5)))
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
                                block5,
                                block6,
                                end,
                                endPID()
                        ))
        );


    }

}