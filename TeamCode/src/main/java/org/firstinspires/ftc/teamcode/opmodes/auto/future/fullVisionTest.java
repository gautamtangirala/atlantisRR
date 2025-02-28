package org.firstinspires.ftc.teamcode.opmodes.auto.future;


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
import org.firstinspires.ftc.teamcode.rrfiles.MecanumDrive;
import org.firstinspires.ftc.teamcode.rrfiles.PinpointDrive;


@Autonomous (name = "full vision testing", preselectTeleOp = "atlantisTele")
public class fullVisionTest extends atlantisAutoEssentials {


    @Override
    public void runOpMode() {


        initDrive();
        initSubsystems();
        initLimelight();

        double fieldOffset = 0; //Change based on ridges of field
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        PinpointDrive drive = new PinpointDrive(hardwareMap, startPose);

        double dropX = -57, dropY = -57, dropAngle = Math.toRadians(45);

        Vector2d dropVector = new Vector2d(dropX, dropY);
        Pose2d dropPose = new Pose2d(dropX, dropY, dropAngle);






        depositGrab(depositClawClose);
        depositTransfer.setPosition(0.5);
        intakeTransfer.setPosition(intakeTransferIn);
        intakeClawTilt.setPosition(0.85);
        horizSetPoint = 0;
        vertSetPoint = 0;
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(2);



        waitForStart();



        TrajectoryActionBuilder pickup = drive.actionBuilder(startPose)
                .strafeToConstantHeading(new Vector2d(0, inchesToBlock))
                .stopAndAdd(new SequentialAction(new ParallelAction(horizSlideOut(true, ticksToBlock), moveWrist(clawPosBlock)), new SleepAction(0.1), clawPickup(), new SleepAction(0.5), closeIntake(), new SleepAction(0.25)))
                ;


        Actions.runBlocking(
                new ParallelAction( updatePidAction(),
                        new SequentialAction(
                                new SleepAction(1),
                                updateLimelightNeural("red"),
                                new SleepAction(2),
                                endPID()
                        )    )
        );

        Actions.runBlocking(
                new ParallelAction( updatePidAction(),
                        drive.actionBuilder(startPose)
                                .strafeToConstantHeading(new Vector2d(0, -inchesToBlock))
                                .stopAndAdd(new SequentialAction(new ParallelAction(horizSlideOut(true, ticksToBlock), moveWrist(clawPosBlock)), new SleepAction(0.1), clawPickup(), new SleepAction(0.5), closeIntake(), new SleepAction(0.25))).build()
                )
        );

    }

}
