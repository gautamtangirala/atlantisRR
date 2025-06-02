package org.firstinspires.ftc.teamcode.opmodes.auto.old;


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
public class fourSampleSlow extends atlantisAutoEssentials {


    @Override
    public void runOpMode() {


        initDrive();
        initSubsystems();

        Pose2d startPose = new Pose2d(-40, -62.625, Math.toRadians(270));
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
        depositTransfer.setPosition(0.4);
        intakeTransfer.setPosition(intakeTransferIn);


        telemetry.addLine("4 SAMPLE AUTO: 64 POINTS");
        telemetry.addLine("ALIGN ROBOT SIDE WITH NET ZONE");
        telemetry.update();
        waitForStart();


        Actions.runBlocking(
                new SequentialAction(
                        dropPreload,
                        block2,
                        block3,
                        block4,
                        end
                )
        );

    }


}
