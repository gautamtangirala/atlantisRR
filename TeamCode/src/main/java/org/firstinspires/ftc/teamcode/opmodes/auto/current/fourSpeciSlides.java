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


@Autonomous (name = "Four Specimen", preselectTeleOp = "atlantisTele")
public class fourSpeciSlides extends atlantisAutoEssentials {


    @Override
    public void runOpMode() {


        initDrive();
        initSubsystems();

        Pose2d startPose = new Pose2d(9, -60, Math.toRadians(270));
        PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);


        //Vars
        double reachOutSlideTiltPos = 0.13;
        //Basket Position XY
        double dropX = 0, dropY = -32.75, dropAngle = Math.toRadians(270);

        double speciIncrement = 1.75; //How far from last speci to drop

        Vector2d dropVector = new Vector2d(dropX, dropY);
        Pose2d dropPose = new Pose2d(dropX, dropY, dropAngle);

        double pickupX = 38, pickupY = -56, pickupAngle = Math.toRadians(90);

        Vector2d pickupVector = new Vector2d(pickupX, pickupY);

        Pose2d pickupPose = new Pose2d(pickupX, pickupY, pickupAngle);


        Action hangPreload = drive.actionBuilder(startPose)
                .afterTime(0, moveSlideHighRung())
                .strafeToLinearHeading(dropVector, dropAngle)
                .stopAndAdd(new SequentialAction(placeSpecimenSlides(),new SleepAction(0.1),openDeposit(), depositTransferAction(0.5)))
                .build();

        Action push2BlocksPickup1 = drive.actionBuilder(dropPose)
                .afterTime(0, moveSlideSpeciPickup())
                .splineToLinearHeading(new Pose2d(35, -30, Math.toRadians(90)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(46, -12.5), Math.toRadians(270))
                .strafeToConstantHeading(new Vector2d(46, -48))
                .strafeToConstantHeading(new Vector2d(48, -15))
                .splineToConstantHeading(new Vector2d(58, -15), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(59, -35), Math.toRadians(270))
                .afterTime(0, depositTransferAction(depositTransferOut))
                .splineToConstantHeading(new Vector2d(38, -45), Math.toRadians(270))
                .waitSeconds(0.25)
                .strafeToConstantHeading(new Vector2d(pickupX, pickupY+0.5))
                .waitSeconds(0.25)
                .stopAndAdd(closeDeposit())
                .waitSeconds(0.25)
                //             .splineToLinearHeading(pickupPose, Math.toRadians(270))
                .build();


        Action hang2 = drive.actionBuilder(pickupPose)

                .afterTime(0, moveSlideHighRung())
                .strafeToLinearHeading(new Vector2d(dropX - 3 , dropY-6), dropAngle)
                .strafeToLinearHeading(new Vector2d(dropX - 3, dropY), dropAngle)
                .stopAndAdd(new SequentialAction(placeSpecimenSlides(),new SleepAction(0.05),openDeposit(), depositTransferAction(0.5)))
                .build();

        Action pickup2 = drive.actionBuilder(new Pose2d(dropX - 3, dropY, dropAngle))
                .afterTime(0, moveSlideSpeciPickup())
                .afterTime(1, depositTransferAction(depositTransferOut))
                .strafeToLinearHeading(new Vector2d(38, -53), Math.toRadians(90))
                .strafeTo(pickupVector)
                .waitSeconds(0.25)
                .stopAndAdd(closeDeposit())
                .waitSeconds(0.25)
                .build();

        Action hang3 = drive.actionBuilder(pickupPose)
                .afterTime(0, moveSlideHighRung())
                .strafeToLinearHeading(new Vector2d(dropX  + speciIncrement, dropY-6), dropAngle)
                .strafeToLinearHeading(new Vector2d(dropX + speciIncrement, dropY), dropAngle)
                .stopAndAdd(new SequentialAction(placeSpecimenSlides(),new SleepAction(0.05),openDeposit(), depositTransferAction(0.5)))
                .build();

        Action pickup3 = drive.actionBuilder(new Pose2d(dropX + speciIncrement, dropY, dropAngle))
                .afterTime(0, moveSlideSpeciPickup())
                .afterTime(1, depositTransferAction(depositTransferOut))
                .strafeToLinearHeading(new Vector2d(38, -53), Math.toRadians(90))
                .strafeTo(pickupVector)
                .waitSeconds(0.25)
                .stopAndAdd(closeDeposit())
                .waitSeconds(0.25)
                .build();

        Action hang4 = drive.actionBuilder(pickupPose)
                .afterTime(0, moveSlideHighRung())
                .strafeToLinearHeading(new Vector2d(dropX + speciIncrement*3, dropY-6), dropAngle)
                .strafeToLinearHeading(new Vector2d(dropX + speciIncrement*3, dropY), dropAngle)
                .stopAndAdd(new SequentialAction(placeSpecimenSlides(),new SleepAction(0.05),openDeposit(), depositTransferAction(0.5)))
                .build();


        Action park = drive.actionBuilder((new Pose2d(dropX + speciIncrement + speciIncrement, dropY, dropAngle)))
                .afterTime(0, moveSlideBottom())
                .strafeToConstantHeading(new Vector2d(pickupX + 10, pickupY + 3))
                .build();

        depositGrab(depositClawClose);
        depositTransfer.setPosition(0.5);
        intakeTransfer.setPosition(intakeTransferIn);

        telemetry.addLine("4 SPECIMEN AUTO: 80 POINTS");
        telemetry.addLine("ALIGN ROBOT SIDE WITH MIDDLE TILE");
        telemetry.addLine("ENSURE SPECIMEN POSITION IS CORRECT");
        telemetry.update();

        waitForStart();


        Actions.runBlocking(
                new ParallelAction(updatePidAction(),
                new SequentialAction(
                        hangPreload,
                        push2BlocksPickup1,
                        hang2,
                        pickup2,
                        hang3,
                        pickup3,
                        hang4,
                        park
                ))
        );

    }


}
