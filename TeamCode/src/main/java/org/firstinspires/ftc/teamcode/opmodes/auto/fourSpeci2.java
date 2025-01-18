package org.firstinspires.ftc.teamcode.opmodes.auto;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.rrfiles.PinpointDrive;

import java.util.Arrays;


@Autonomous //(preselectTeleOp = "tele")
public class fourSpeci2 extends atlantisAutoEssentials {



    @Override
    public void runOpMode() {


        initDrive();
        initSubsystems();

        Pose2d startPose = new Pose2d(9, -60, Math.toRadians(270));
        PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);


        //Vars
        double reachOutSlideTiltPos = 0.13;
        //Basket Position XY
        double dropX = 2, dropY = -29, dropAngle = Math.toRadians(270);

        double speciIncrement = 2; //How far from last speci to drop

        Vector2d dropVector = new Vector2d(dropX, dropY);
        Pose2d dropPose = new Pose2d(dropX, dropY, dropAngle);

        double pickupX = 36, pickupY = -56, pickupAngle = Math.toRadians(90);

        Vector2d pickupVector = new Vector2d(pickupX, pickupY);

        Pose2d pickupPose = new Pose2d(pickupX, pickupY, pickupAngle);




        Action hangPreload = drive.actionBuilder(startPose)
                .strafeToLinearHeading(dropVector, dropAngle)
                .build();

        Action push2Blocks = drive.actionBuilder(dropPose)
                .afterTime(0.5, moveSlideBottom())
                .splineToLinearHeading(new Pose2d(35, -30, Math.toRadians(90)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(46,-12.5),Math.toRadians(270))
                .strafeToConstantHeading(new Vector2d(46, -48))
                .strafeToConstantHeading(new Vector2d(48, -10))
                .strafeToConstantHeading(new Vector2d(60, -15))
                .strafeToConstantHeading(new Vector2d(60, -48))
                .strafeTo(new Vector2d(36, -42.5))
                .strafeTo(pickupVector)
   //             .splineToLinearHeading(pickupPose, Math.toRadians(270))
                .build();

        Action pickup = drive.actionBuilder(dropPose)
                .strafeTo(new Vector2d(36, -42.5))
                .strafeTo(pickupVector).build();

        Action hang = drive.actionBuilder(pickupPose)
                .strafeToLinearHeading(dropVector, dropAngle)
                .build();



        Action hangPark = drive.actionBuilder(pickupPose)
                .splineToLinearHeading(new Pose2d(2, dropY, Math.toRadians(270)), Math.toRadians(90))
                .afterTime(0, moveSlideSpeci())
                .stopAndAdd(placeSpecimen())
                .stopAndAdd(openDeposit())
                .splineToLinearHeading(pickupPose, Math.toRadians(270))
                .afterTime(0.25, moveSlideBottom())
                .build();

        depositGrab(depositClawClose);
        depositTransfer.setPosition(0.5);
        intakeTransfer.setPosition(intakeTransferIn);

        waitForStart();



        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(hangPreload, moveSlideSpeci()),
                        placeSpecimen(),
                        new SleepAction(0.25),
                        openDeposit(),
                        push2Blocks,
                        new SleepAction(0.25),
                        closeDeposit(),
                        new SleepAction(0.25),
                        new ParallelAction(hang, moveSlideSpeci()),
                        placeSpecimen(),
                        new SleepAction(0.25),
                        openDeposit()

   //                     hangSpeciPickupNext,
     //                   hangSpeciPickupNext,
       //                 hangPark

                )
        );

    }


}
