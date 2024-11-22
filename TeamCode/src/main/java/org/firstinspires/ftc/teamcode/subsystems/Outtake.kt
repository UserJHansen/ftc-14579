package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.ftc.DownsampledWriter
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.ftclib.PIDFController
import org.firstinspires.ftc.teamcode.galahlib.StateLoggable
import org.firstinspires.ftc.teamcode.galahlib.actions.Loggable
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggableAction
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggingSequential
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.CurrentCutoff
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.LinkedLift
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.ServoMultiState
import org.firstinspires.ftc.teamcode.messages.StringMessage

@Config
class Outtake(hardwareMap: HardwareMap) : StateLoggable {
    companion object PARAMS {
        @JvmField
        var PIDController = PIDFController(
            10.0,
            0.0,
            0.0,
            0.0
        )

        @JvmField
        var SpecimenCurrentTrigger = 8.0

        @JvmField
        var resetCurrentCutoff = 5.0

        class LiftPositions {
            @JvmField
            var topBasket = 27.0

            @JvmField
            var topSpecimen = 12.5
        }

        class GrabberPositions {
            @JvmField
            var waiting = 0.45

            @JvmField
            var close = 0.3

            @JvmField
            var grabbing = 0.22
        }

        class PivotPositions {
            @JvmField
            var intake = 0.03

            @JvmField
            var specimenCapture = 0.97

            @JvmField
            var specimen = 0.47

            @JvmField
            var deposit = 0.34

            @JvmField
            var depositDown = 0.27
        }

        val liftPositions = LiftPositions()
        val grabberPositions = GrabberPositions()
        val pivotPositions = PivotPositions()
    }

    val lift =
        LinkedLift(hardwareMap, "slide1", "slide2", PIDController, DcMotorSimple.Direction.FORWARD)

    val grabber = ServoMultiState(
        hardwareMap, "outtakeClaw",
        doubleArrayOf(
            grabberPositions.waiting,
            grabberPositions.close,
            grabberPositions.grabbing
        ), 0.4
    )
    val pivot = ServoMultiState(
        hardwareMap, "outtakePivot",
        doubleArrayOf(
            pivotPositions.intake,
            pivotPositions.specimenCapture,
            pivotPositions.specimen,
            pivotPositions.deposit,
            pivotPositions.depositDown
        ), 1.17
    )

    private val outtakeActionWriter = DownsampledWriter("OUTTAKE_ACTION", 50_000_000)

    fun resetLifts(): LoggableAction {
        outtakeActionWriter.write(StringMessage("RESET_LIFTS"))
        return lift.resetPosition(
            CurrentCutoff(lift.firstLift).above(resetCurrentCutoff)
        )
    }

    fun homePosition(): LoggableAction {
        return Loggable(
            "MOVE_ARM_TRANSFER", SequentialAction(
                InstantAction {
                    outtakeActionWriter.write(StringMessage("MOVE_ARM_TRANSFER"))
                },
                grabber.setPosition(1),
                pivot.setPosition(0),
            )
        )
    }

    fun pickupInternalSample(): LoggableAction {
        return Loggable("GRAB_SAMPLE", grabber.setPosition(2))
    }

    fun topBasket(): LoggableAction {
        return LoggingSequential(
            "GOTO_TOP_BASKET",
            Loggable("LOG_ACTION", InstantAction {
                outtakeActionWriter.write(StringMessage("TOP_BASKET"))
            }),
            lift.goToThroughWhile(
                liftPositions.topBasket,
                liftPositions.topBasket / 2,
                Loggable(
                    "PIVOT_OUTTAKE",
                    pivot.setPosition(3)
                )
            ),
            Loggable(
                "LOCK_IN",
                pivot.setPosition(4)
            )
        )
    }

    fun dropSample(): LoggableAction {
        return LoggingSequential(
            "RELEASE_SAMPLE", Loggable("LOG", InstantAction {
                outtakeActionWriter.write(StringMessage("DROP_SAMPLE"))
            }),
            Loggable("LET_GO", grabber.setPosition(0)),
            Loggable("WAIT_FOR_DROP", SleepAction(0.5)),
            Loggable("GET_OUT", pivot.setPosition(3))
        )
    }

    fun retractArm(): LoggableAction = LoggingSequential(
        "RETRACT_ARM",
        Loggable("LOG", InstantAction {
            outtakeActionWriter.write(StringMessage("RETRACT"))
        }),

        Loggable(
            "MOVE_ARM_IN", ParallelAction(
                grabber.setPosition(1),
                pivot.setPosition(0)
            )
        ),
        lift.gotoDistance(0.0)
    )

    fun specimenReady(open: Boolean): LoggableAction {
        return LoggingSequential(
            "SPECIMEN_READY",
            StaticLights.setColours(
                arrayOf(
                    RevBlinkinLedDriver.BlinkinPattern.WHITE,
                    RevBlinkinLedDriver.BlinkinPattern.GREEN
                )
            ),
            Loggable("LOG_ACTION", InstantAction {
                outtakeActionWriter.write(StringMessage("SPECIMEN_READY"))
            }),
            lift.gotoDistance(4.0),
            Loggable(
                "MOVE_ARM_OUT", ParallelAction(
                    grabber.setPosition(2),
                    pivot.setPosition(1)
                )
            ),
            lift.gotoDistance(0.0),
            Loggable("MOVE_GRABBER_TO_POSITION", grabber.setPosition(if (open) 0 else 2))
        )
    }

    fun grabber(open: Boolean): LoggableAction {
        return LoggingSequential(
            "MOVE_GRABBER",
            Loggable("MOVE_GRABBER", ParallelAction(InstantAction {
                outtakeActionWriter.write(StringMessage("MOVE_GRABBER"))
            }, grabber.setPosition(if (open) 0 else 2))),
        )
    }

    fun raiseSpecimen(): LoggableAction {
        return LoggingSequential(
            "GRAB_SPECIMEN",
            Loggable("GRAB_SPECIMEN", InstantAction {
                outtakeActionWriter.write(StringMessage("GRAB_SPECIMEN"))
            }),
            lift.goToThroughWhile(
                liftPositions.topSpecimen,
                liftPositions.topSpecimen / 2,
                Loggable(
                    "MOVE_ARM_OUT", ParallelAction(
                        pivot.setPosition(2),
                        SleepAction(0.2),
                    )
                ),
            ),
        )
    }

    fun placeSpecimen(): LoggableAction {
        return LoggingSequential(
            "PLACE_SPECIMEN",
            lift.gotoDistance(liftPositions.topSpecimen),
            Loggable("LOG_ACTION", InstantAction {
                outtakeActionWriter.write(StringMessage("PLACE_SPECIMEN"))
                lift.lockedOut = true
                lift.forBothMotors { motor ->
                    motor.power = 1.0
                }
            }),
            lift.gotoDistance(liftPositions.topSpecimen + 6, 0.25),
            Loggable(
                "PID_NORMAL",
                InstantAction {
                    lift.lockedOut = false
                    lift.forBothMotors { motor ->
                        motor.power = 0.0
                    }
                }),
        )
    }

    fun ensureSpecimenPlaced(): LoggableAction {
        return object : LoggableAction {
            override val name: String
                get() = if (currentAction != null) currentAction!!.name else "SPECIMEN_PLACE"
            var currentAction: LoggableAction? = null;
            var currentTriggered = false
            val currentTriggerAction = SequentialAction(
                CurrentCutoff(lift.firstLift).above(SpecimenCurrentTrigger),
                CurrentCutoff(lift.firstLift).below(SpecimenCurrentTrigger)
            )

            override fun run(p: TelemetryPacket): Boolean {
                var complete = false
                if (currentAction == null || !currentAction!!.run(p)) {
                    currentAction = placeSpecimen()
                    complete = true
                }

                if (!currentTriggered) {
                    currentTriggered = !currentTriggerAction.run(p)
                }

                return !currentTriggered || !complete
            }

        }
    }

    fun returnSpecimen(): LoggableAction {
        return LoggingSequential(
            "RETURN_SPECIMEN",
            Loggable(
                "LET_GO", ParallelAction(
                    grabber.setPosition(0),
                    pivot.setPosition(0)
                )
            ),
            Loggable(
                "UNLOCK_FROM_SPECIMEN",
                InstantAction {
                    lift.lockedOut = false
                }),
            lift.gotoDistance(0.0),
            StaticLights.setColours(
                arrayOf(
                    RevBlinkinLedDriver.BlinkinPattern.BLACK
                )
            ),
        )
    }

    fun abortSpecimen(): LoggableAction {
        return LoggingSequential(
            "ABORT_SPECIMEN",
            Loggable("LOG_ACTION", InstantAction {
                outtakeActionWriter.write(StringMessage("ABORT_SPECIMEN"))
            }),
            Loggable(
                "UNLOCK_FROM_SPECIMEN",
                InstantAction {
                    lift.lockedOut = false
                }),
            lift.gotoDistance(4.0),
            Loggable(
                "MOVE_SAFE", ParallelAction(
                    grabber.setPosition(1),
                    pivot.setPosition(0)
                )
            ),
            lift.gotoDistance(0.0),

            StaticLights.setColours(
                arrayOf(
                    RevBlinkinLedDriver.BlinkinPattern.BLACK
                )
            )
        )
    }

    override fun logState(uniqueName: String?) {
        lift.logState("$uniqueName [OUTTAKE_SLIDES]")
    }

    fun lockout() {
        lift.lockout()
    }

    fun unlock() {
        lift.unlock()
    }
}
