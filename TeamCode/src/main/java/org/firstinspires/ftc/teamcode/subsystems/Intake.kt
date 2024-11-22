package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SleepAction
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.internal.system.Deadline
import org.firstinspires.ftc.teamcode.galahlib.StateLoggable
import org.firstinspires.ftc.teamcode.galahlib.actions.Loggable
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggableAction
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggingSequential
import org.firstinspires.ftc.teamcode.galahlib.actions.race
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.ContinuousServo
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.CurrentCutoff
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.DigitalInput
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.Lift
import org.firstinspires.ftc.teamcode.galahlib.mechanisms.ServoToggle
import org.firstinspires.ftc.teamcode.messages.ColorMessage
import org.firstinspires.ftc.teamcode.staticData.Logging
import org.firstinspires.ftc.teamcode.staticData.PoseStorage
import java.util.concurrent.TimeUnit
import kotlin.math.max

@Config
class Intake(hardwareMap: HardwareMap) : StateLoggable {
    companion object PARAMS {
        @JvmField // Forward intake power
        var P_Intake: Double = 10.0

        @JvmField
        var transferDuration = 2.0

        @JvmField
        var captureTimeout = 100L

        @JvmField
        var resetCurrentCutoff = 5.0

        @JvmField
        var minExtension = 4.0

        @JvmField
        var maxExtension = 9.5420424719

        class FlipLimits {
            @JvmField
            var downPosition = 0.36

            @JvmField
            var upPosition = 0.04
        }

        @JvmField
        val flipLimits = FlipLimits()
    }

    val slides = Lift(
        hardwareMap,
        "extendo",
        DcMotorSimple.Direction.FORWARD,
        P_Intake,
        85.935483871
    )

    val flipServo = ServoToggle(
        hardwareMap,
        "extendoClaw",
        flipLimits.upPosition,
        flipLimits.downPosition,
        1.5
    )
    val pullServo = ContinuousServo(
        hardwareMap,
        "extendoSuck",
        DcMotorSimple.Direction.FORWARD
    )
    val colorSensor = hardwareMap.get(RevColorSensorV3::class.java, "intakeColour")
    val intakeTouch = DigitalInput(hardwareMap, "intake")

    init {
        (flipServo.servo as ServoImplEx).pwmRange = PwmControl.PwmRange(500.0, 2500.0)
    }

    val sampleType: SampleType
        get() {
            val color = this.colorSensor.normalizedColors
            val max = max(color.red, max(color.green, color.blue))

            color.red /= max
            color.green /= max
            color.blue /= max

            Logging.DEBUG("Current colour", ColorMessage(color))

            return if (color.red == 1f) {
                SampleType.Red
            } else if (color.blue == 1f) {
                SampleType.Blue
            } else if (color.green == 1f) {
                SampleType.Shared
            } else {
                SampleType.Unknown
            }
        }
    val distanceTriggered: Boolean
        get() {
            return colorTriggered || intakeTouch.triggered
        }
    val colorTriggered: Boolean
        get() {
            val distance = colorSensor.getDistance(DistanceUnit.MM)
            Logging.DEBUG("FRONT_DISTANCE", distance)
            return distance < 20
        }

    fun resetSlides(): LoggableAction {
        return slides.resetPosition(
            CurrentCutoff(slides.liftMotor).above(resetCurrentCutoff)
        )
    }

    fun pickSampleForward(shared: Boolean): LoggableAction {
        return object : LoggableAction {
            var captureTimeout: Deadline? = null
            var releaseTimeout: Deadline? = null
            override val name: String
                get() = "PICKING_SAMPLE_${if (captureTimeout == null) "SEARCHING" else "WAITING"}"
            var initialized = false
            override fun run(p: TelemetryPacket): Boolean {
                if (!initialized) {
                    Logging.LOG("PICK_SAMPLE")
                    initialized = true
                }

                val currentSample = sampleType
                val allianceColor =
                    if (PoseStorage.isRedAlliance) SampleType.Red else SampleType.Blue

                if (releaseTimeout != null) {
                    captureTimeout = null
                    if (releaseTimeout!!.hasExpired())
                        releaseTimeout = null
                } else if (!distanceTriggered) { // No Sample
                    captureTimeout = null
                    pullServo.servo.power = 1.0
                } else if (currentSample == allianceColor || (shared && currentSample == SampleType.Shared) && intakeTouch.triggered) { // Good Sample caught
                    pullServo.servo.power = 0.0
                    StaticLights.colors[1] = when (currentSample) {
                        SampleType.Red -> RevBlinkinLedDriver.BlinkinPattern.RED
                        SampleType.Blue -> RevBlinkinLedDriver.BlinkinPattern.BLUE
                        SampleType.Shared -> RevBlinkinLedDriver.BlinkinPattern.YELLOW
                        else -> RevBlinkinLedDriver.BlinkinPattern.BLACK
                    }
                    if (captureTimeout == null)
                        captureTimeout = Deadline(PARAMS.captureTimeout, TimeUnit.MILLISECONDS)
                } else { // Bad Sample
                    captureTimeout = null
                    pullServo.servo.power = -1.0
                    StaticLights.colors[1] = RevBlinkinLedDriver.BlinkinPattern.BLACK
                }

                if (captureTimeout?.hasExpired() == true || PoseStorage.shouldHallucinate) {
                    pullServo.servo.power = 0.0

                    return false
                }
                return true
            }
        }
    }

    fun retractSlides(): LoggableAction {
        return LoggingSequential(
            "RETRACT_FLIP_INTAKE",
            Loggable(
                "RETRACT_AND_FLIP", ParallelAction(
                    slides.gotoDistance(0.0, 0.1),
                    flipServo.setPosition(false),
                )
            ),
        )
    }

    fun transfer(): LoggableAction {
        return Loggable("TRANSFER", ParallelAction(
            pullServo.setSpeed(1.0),
            SleepAction(transferDuration)
        ))
    }

    @JvmOverloads
    fun captureSample(
        shared: Boolean,
        positionProvider: Lift.DoubleProvider? = null
    ): LoggableAction {
        if (positionProvider?.run() == null) return LoggingSequential(
            "CAPTURE_CLOSE_SAMPLE",

            StaticLights.setColours(arrayOf(
                RevBlinkinLedDriver.BlinkinPattern.WHITE,
                RevBlinkinLedDriver.BlinkinPattern.BLACK,
            )),
            Loggable(
                "SEARCH_AND_FLIP", ParallelAction(
                    Loggable("FLIP_INTAKE_DOWN", flipServo.setPosition(true)),
                    slides.gotoDistance(0.0),
                    pickSampleForward(shared),
                )
            ),
        )

        val holdPositionAction = slides.holdVariablePosition(positionProvider)
        val flipIntakeAction = flipServo.setPosition(true)
        var flipComplete = false

        return LoggingSequential(
            "CAPTURE_FAR_SAMPLE",
            StaticLights.setColours(arrayOf(
                RevBlinkinLedDriver.BlinkinPattern.WHITE,
                RevBlinkinLedDriver.BlinkinPattern.BLACK,
            )),
            Loggable(
                "SEARCH_AND_FIND", ParallelAction(
                    race(
                        fun (p: TelemetryPacket): Boolean {
                            if (!flipComplete && slides.currentPosition > minExtension) {
                                flipComplete = !flipIntakeAction.run(p)
                            }

                            return holdPositionAction.run(p)
                        },
                        pickSampleForward(shared)
                    ),
                )
            ),
            retractSlides(),
        )
    }

    override fun logState(uniqueName: String?) {
        slides.logState("$uniqueName [INTAKE_SLIDES]")
    }

    fun lockout() {
        slides.lockout()
    }

    fun unlock() {
        slides.unlock()
    }
}
