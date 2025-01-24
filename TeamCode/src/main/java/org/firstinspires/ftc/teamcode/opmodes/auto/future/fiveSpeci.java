package org.firstinspires.ftc.teamcode.opmodes.auto.future;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.opmodes.auto.atlantisAutoEssentials;
import org.firstinspires.ftc.teamcode.rrfiles.PinpointDrive;



@Disabled
@Autonomous (preselectTeleOp = "atlantisTele")
public class fiveSpeci extends atlantisAutoEssentials {


    @Override
    public void runOpMode() {


        initDrive();
        initSubsystems();

        Pose2d startPose = new Pose2d(9, -60, Math.toRadians(270));
        PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);


        //Vars
        double reachOutSlideTiltPos = 0.13;
        //Basket Position XY
        double dropX = 0, dropY = -29, dropAngle = Math.toRadians(270);

        double speciIncrement = 1.75; //How far from last speci to drop

        Vector2d dropVector = new Vector2d(dropX, dropY);
        Pose2d dropPose = new Pose2d(dropX, dropY, dropAngle);

        double pickupX = 38, pickupY = -56.5, pickupAngle = Math.toRadians(90);

        Vector2d pickupVector = new Vector2d(pickupX, pickupY);

        Pose2d pickupPose = new Pose2d(pickupX, pickupY, pickupAngle);


        Action hangPreload = drive.actionBuilder(startPose)
                .afterTime(0, moveSlideSpeci())
                .strafeToLinearHeading(dropVector, dropAngle)
                .stopAndAdd(placeSpecimen())
                .waitSeconds(0.25)
                .stopAndAdd(openDeposit())
                .splineToLinearHeading(new Pose2d(35, -30, Math.toRadians(90)), Math.toRadians(90))
                .build();

        Action push3BlocksPickup1 = drive.actionBuilder(new Pose2d(35, -30, Math.toRadians(90)))
                .afterTime(0, moveSlideBottom())
                .afterTime(0, depositTransferAction(depositTransferOut))
                .splineToConstantHeading(new Vector2d(46, -10), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(46, -48), Math.toRadians(270))

                .splineToConstantHeading(new Vector2d(50, -10), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(51, -10), Math.toRadians(0))

                .splineToConstantHeading(new Vector2d(53, -48), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(58, -10), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(59, pickupY), Math.toRadians(270))
                .waitSeconds(0.25)
                .stopAndAdd(closeDeposit())
                .waitSeconds(0.25)
                //             .splineToLinearHeading(pickupPose, Math.toRadians(270))
                .build();


        Action hang2 = drive.actionBuilder(new Pose2d(59, pickupY, Math.toRadians(90)))
                .afterTime(0, moveSlideSpeci())

                .strafeToLinearHeading(new Vector2d(dropX - speciIncrement, dropY), dropAngle)
                .stopAndAdd(placeSpecimen())
                .waitSeconds(0.25)
                .stopAndAdd(openDeposit())
                .build();

        Action pickup2 = drive.actionBuilder(new Pose2d(dropX - speciIncrement, dropY, dropAngle))
                .afterTime(0, moveSlideBottom())
                .afterTime(1, depositTransferAction(depositTransferOut))
                .strafeToLinearHeading(new Vector2d(36, -53), Math.toRadians(90))
                .strafeTo(pickupVector)
                .waitSeconds(0.25)
                .stopAndAdd(closeDeposit())
                .waitSeconds(0.25)
                .build();

        Action hang3 = drive.actionBuilder(pickupPose)
                .afterTime(0, moveSlideSpeci())
                .strafeToLinearHeading(new Vector2d(dropX + speciIncrement, dropY), dropAngle)
                .stopAndAdd(placeSpecimen())
                .waitSeconds(0.25)
                .stopAndAdd(openDeposit())
                .build();

        Action pickup3 = drive.actionBuilder(new Pose2d(dropX + speciIncrement, dropY, dropAngle))
                .afterTime(0, moveSlideBottom())
                .afterTime(1, depositTransferAction(depositTransferOut))
                .strafeToLinearHeading(new Vector2d(36, -53), Math.toRadians(90))
                .strafeTo(pickupVector)
                .waitSeconds(0.25)
                .stopAndAdd(closeDeposit())
                .waitSeconds(0.25)
                .build();

        Action hang4 = drive.actionBuilder(pickupPose)
                .afterTime(0, moveSlideSpeci())
                .strafeToLinearHeading(new Vector2d(dropX + speciIncrement + speciIncrement, dropY), dropAngle)
                .stopAndAdd(placeSpecimen())
                .waitSeconds(0.25)
                .stopAndAdd(openDeposit())
                .build();


        Action pickup4 = drive.actionBuilder(new Pose2d(dropX + speciIncrement + speciIncrement, dropY, dropAngle))
                .afterTime(0, moveSlideBottom())
                .afterTime(1, depositTransferAction(depositTransferOut))
                .strafeToLinearHeading(new Vector2d(36, -53), Math.toRadians(90))
                .strafeTo(pickupVector)
                .waitSeconds(0.25)
                .stopAndAdd(closeDeposit())
                .waitSeconds(0.25)
                .build();

        Action hang5 = drive.actionBuilder(pickupPose)
                .afterTime(0, moveSlideSpeci())
                .strafeToLinearHeading(new Vector2d(dropX - speciIncrement - speciIncrement , dropY), dropAngle)
                .stopAndAdd(placeSpecimen())
                .waitSeconds(0.25)
                .stopAndAdd(openDeposit())
                .build();





        Action park = drive.actionBuilder((new Pose2d(dropX - speciIncrement - speciIncrement, dropY, dropAngle)))
                .afterTime(0, moveSlideBottom())
                .strafeToConstantHeading(new Vector2d(pickupX + 10, pickupY + 3))
                .build();

        depositGrab(depositClawClose);
        depositTransfer.setPosition(0.5);
        intakeTransfer.setPosition(intakeTransferIn);

        waitForStart();


        Actions.runBlocking(
                new SequentialAction(
                        hangPreload,
                        push3BlocksPickup1,
                        hang2,
                        pickup2,
                        hang3,
                        pickup3,
                        hang4,
                        pickup4,
                        hang5,
                        park
                )
        );

    }


}





