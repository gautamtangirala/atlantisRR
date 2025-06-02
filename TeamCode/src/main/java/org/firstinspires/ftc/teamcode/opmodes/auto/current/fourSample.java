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


@Autonomous (name = "Four Sample With Park", preselectTeleOp = "atlantisTele")
public class fourSample extends atlantisAutoEssentials {


    @Override
    public void runOpMode() {


        initDrive();
        initSubsystems();

        double fieldOffset = 0; //Change based on ridges of field
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
                .afterTime(0.5,moveSlidePark())
                .afterTime(1, depositTransferAction(0.7))
                .splineToLinearHeading(new Pose2d(-25, -10, Math.toRadians(180)), Math.toRadians(0))
                .waitSeconds(0.1)
                .build();


        depositGrab(depositClawClose);
        depositTransfer.setPosition(0.4);
        intakeTransfer.setPosition(intakeTransferIn);
        intakeClawTilt.setPosition(0.85);
        intakeClawWrist.setPosition(intakeWristVert);

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
                                end,
                                endPID()
                        ))
        );

    }

}