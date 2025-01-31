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


@Autonomous (name = "Four Sample", preselectTeleOp = "atlantisTele")
public class fourSampleQuick extends atlantisAutoEssentials {


    @Override
    public void runOpMode() {


        initDrive();
        initSubsystems();

        double fieldOffset = 0; //Change based on ridges of field
        Pose2d startPose = new Pose2d(-40, -62.625 + fieldOffset, Math.toRadians(270));
        PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);

        double dropX = -57, dropY = -56.5, dropAngle = Math.toRadians(45);

        Vector2d dropVector = new Vector2d(dropX, dropY);
        Pose2d dropPose = new Pose2d(dropX, dropY, dropAngle);






        Action dropPreload = drive.actionBuilder(startPose)
                .afterTime(0, moveSlideHighBasket())
                .afterTime(0, openIntake())

                .strafeToLinearHeading(dropVector, dropAngle)
                .stopAndAdd( new SequentialAction(moveSlideHighBasket(), depositTransferAction(0.7), new SleepAction(0.25), openDeposit(), new SleepAction(0.25), depositTransferAction(0.5)))
                .build();

        Action block2 = drive.actionBuilder(dropPose)
                .afterTime(0.1,moveSlideBottom())
                .strafeToLinearHeading(new Vector2d(-49.25, -50.75), Math.toRadians(90))
                .stopAndAdd(new SequentialAction(horizSlideOut(false), new SleepAction(0.1), clawPickup(), new SleepAction(0.7), closeIntake(), new SleepAction(0.1)))

                //transfer
                .afterTime(0, transfer(1))
                .afterTime(0.5, transfer(2))
                .afterTime(1, closeDeposit())
                .afterTime(1.1, openIntake())
                .afterTime(1.35, transfer(3))



                //drop block
                .afterTime(1.35, new SequentialAction(moveSlideHighBasket(), depositTransferAction(0.7), new SleepAction(0.25), openDeposit(), new SleepAction(0.25), depositTransferAction(0.5)))
                .strafeToLinearHeading(dropVector, dropAngle)
                .build();

        Action block3 = drive.actionBuilder(dropPose)
                .afterTime(0.1,moveSlideBottom())
                .strafeToLinearHeading(new Vector2d(-59.25, -50.75), Math.toRadians(90))
                .stopAndAdd(new SequentialAction(horizSlideOut(false), new SleepAction(0.1), clawPickup(), new SleepAction(0.7), closeIntake(), new SleepAction(0.1)))

                //transfer
                .afterTime(0, transfer(1))
                .afterTime(0.5, transfer(2))
                .afterTime(1, closeDeposit())
                .afterTime(1.1, openIntake())
                .afterTime(1.35, transfer(3))

                //drop block
                .afterTime(1.35, new SequentialAction(moveSlideHighBasket(), depositTransferAction(0.7), new SleepAction(0.25), openDeposit(), new SleepAction(0.25), depositTransferAction(0.5)))
                .strafeToLinearHeading(dropVector, dropAngle)
                .build();

        Action block4 = drive.actionBuilder(dropPose)
                .afterTime(0.1,moveSlideBottom())
                .strafeToLinearHeading(new Vector2d(-45.25, -27), Math.toRadians(180))
                .stopAndAdd( new SequentialAction( new ParallelAction(horizSlideOut(false), moveWrist(0)), new SleepAction(0.1), clawPickup(), new SleepAction(0.7), closeIntake(), new SleepAction(0.1)))

                //transfer
                .afterTime(0, transfer(1))
                .afterTime(0.5, transfer(2))
                .afterTime(1, closeDeposit())
                .afterTime(1.1, openIntake())
                .afterTime(1.35, transfer(3))

                //drop block
                .afterTime(1.35, new SequentialAction(moveSlideHighBasket(), depositTransferAction(0.7), new SleepAction(0.25), openDeposit(), new SleepAction(0.25), depositTransferAction(0.5)))
                .strafeToLinearHeading(dropVector, dropAngle)
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

        waitForStart();



        Actions.runBlocking(
                new ParallelAction( updatePidAction(),
                new SequentialAction(
                        dropPreload,
                        block2,
                        block3,
                        block4,
                        end
                ))
        );

    }

}
