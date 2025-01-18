package org.firstinspires.ftc.teamcode.opmodes.auto;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.rrfiles.PinpointDrive;


@Autonomous //(preselectTeleOp = "tele")
public class fourSpeci extends atlantisAutoEssentials {



    @Override
    public void runOpMode() {


        initDrive();
        initSubsystems();

        Pose2d startPose = new Pose2d(9, -60, Math.toRadians(270));
        PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);


        //Vars
        double reachOutSlideTiltPos = 0.13;
        //Basket Position XY
        double dropX = 2, dropY = -32, dropAngle = Math.toRadians(270);

        double speciIncrement = 2; //How far from last speci to drop

        Vector2d dropVector = new Vector2d(dropX, dropY);
        Pose2d dropPose = new Pose2d(dropX, dropY, dropAngle);

        double pickupX = 35, pickupY = -57, pickupAngle = Math.toRadians(90);

        Vector2d pickupVector = new Vector2d(pickupX, pickupY);

        Pose2d pickupPose = new Pose2d(pickupX, pickupY, pickupAngle);


        Action hangPreload = drive.actionBuilder(startPose)
                .strafeToLinearHeading(dropVector, dropAngle)
                .afterTime(0, moveSlideSpeci())
                .stopAndAdd(placeSpecimen())
                .stopAndAdd(openDeposit())
                .build();

        Action push2Blocks = drive.actionBuilder(dropPose)
                .splineToLinearHeading(new Pose2d(35, -30, Math.toRadians(90)), Math.toRadians(90))
                .afterTime(1.5, moveSlideBottom())
                .splineToLinearHeading(new Pose2d(46, -10, Math.toRadians(90)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(46, -50, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(48, -10, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(58, -10, Math.toRadians(90)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(59, -50, Math.toRadians(90)), Math.toRadians(270))
                .splineToLinearHeading(pickupPose, Math.toRadians(270))
                .build();


        Action hangSpeciPickupNext = drive.actionBuilder(pickupPose)
                .splineToLinearHeading(new Pose2d(2, dropY, Math.toRadians(270)), Math.toRadians(90))
                .afterTime(0, moveSlideSpeci())
                .stopAndAdd(placeSpecimen())
                .stopAndAdd(openDeposit())
                .splineToLinearHeading(pickupPose, Math.toRadians(270))
                .afterTime(0, moveSlideBottom())
                .stopAndAdd(closeDeposit())
                .build();

        Action hangPark = drive.actionBuilder(pickupPose)
                .splineToLinearHeading(new Pose2d(2, dropY, Math.toRadians(270)), Math.toRadians(90))
                .afterTime(0, moveSlideSpeci())
                .stopAndAdd(placeSpecimen())
                .stopAndAdd(openDeposit())
                .splineToLinearHeading(pickupPose, Math.toRadians(270))
                .afterTime(0.25, moveSlideBottom())
                .build();

        waitForStart();
        depositTransfer.setPosition(0.5);
        depositGrab(depositClawClose);
        intakeTransfer.setPosition(intakeTransferIn);


        Actions.runBlocking(
                new SequentialAction(
                        hangPreload,
                        push2Blocks,
                        hangSpeciPickupNext,
                        hangSpeciPickupNext,
                        hangPark

                )
        );

    }


}
