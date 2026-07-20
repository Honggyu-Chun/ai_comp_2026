#!/usr/bin/env python3

import math
import shutil
import subprocess
import threading
import time

import numpy as np
import rospy
from morai_msgs.msg import EgoVehicleStatus
from std_msgs.msg import Float64, Int16, String


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def smoothstep(value):
    value = clamp(value, 0.0, 1.0)
    return value * value * (3.0 - 2.0 * value)


class VirtualPowertrain:
    """Convert vehicle speed into a high-revving virtual engine state."""

    def __init__(self):
        self.max_speed_kph = float(rospy.get_param("~max_speed_kph", 120.0))
        self.idle_rpm = float(rospy.get_param("~idle_rpm", 800.0))
        self.redline_rpm = float(rospy.get_param("~redline_rpm", 7000.0))
        self.soft_limit_rpm = float(rospy.get_param("~soft_limit_rpm", 6800.0))
        self.max_rpm = float(rospy.get_param("~max_rpm", 7200.0))
        self.upshift_rpm = float(rospy.get_param("~upshift_rpm", 6500.0))
        self.downshift_rpm = float(rospy.get_param("~downshift_rpm", 2500.0))
        self.rpm_rise_tau_s = max(0.0, float(rospy.get_param("~rpm_rise_tau_s", 0.030)))
        self.rpm_fall_tau_s = max(0.0, float(rospy.get_param("~rpm_fall_tau_s", 0.050)))
        self.speed_rpm_curve_exponent = clamp(
            float(rospy.get_param("~speed_rpm_curve_exponent", 0.72)), 0.2, 2.0
        )
        self.gear_max_speeds_kph = [float(v) for v in rospy.get_param(
            "~gear_max_speeds_kph", [130.0]
        )]
        if not self.gear_max_speeds_kph:
            self.gear_max_speeds_kph = [60.0]
        self.gear_max_speeds_kph = sorted(max(0.1, v) for v in self.gear_max_speeds_kph)
        self.upshift_duration_s = max(
            0.01, float(rospy.get_param("~upshift_duration_s", 0.12))
        )
        self.downshift_duration_s = max(
            0.01, float(rospy.get_param("~downshift_duration_s", 0.15))
        )
        self.shift_cooldown_s = max(
            0.0, float(rospy.get_param("~shift_cooldown_s", 0.18))
        )
        self.blip_gain = max(0.0, float(rospy.get_param("~blip_gain", 1.0)))
        self.kickdown_throttle = clamp(
            float(rospy.get_param("~kickdown_throttle", 0.85)), 0.0, 1.0
        )
        self.brake_downshift = clamp(
            float(rospy.get_param("~brake_downshift", 0.30)), 0.0, 1.0
        )

        self.gear = 1
        self.rpm = self.idle_rpm
        self.previous_speed_kph = 0.0
        self.pending_gear = None
        self.shift_direction = 0
        self.shift_elapsed_s = 0.0
        self.shift_duration_s = 0.0
        self.shift_start_rpm = self.idle_rpm
        self.shift_cooldown_remaining_s = 0.0
        self.shift_blip = 0.0
        self.torque_scale = 1.0
        self.previous_throttle = 0.0

    def rpm_for_gear(self, speed_kph, gear):
        speed_kph = clamp(float(speed_kph), 0.0, self.max_speed_kph)
        index = clamp(int(gear) - 1, 0, len(self.gear_max_speeds_kph) - 1)
        maximum_speed = self.gear_max_speeds_kph[index]
        speed_ratio = clamp(speed_kph / maximum_speed, 0.0, 1.0)
        return clamp(
            self.idle_rpm
            + (self.redline_rpm - self.idle_rpm)
            * math.pow(speed_ratio, self.speed_rpm_curve_exponent),
            self.idle_rpm,
            self.max_rpm,
        )

    def can_downshift(self, speed_kph, target_gear):
        if target_gear < 1:
            return False
        return self.rpm_for_gear(speed_kph, target_gear) < self.soft_limit_rpm

    def shift_label(self):
        if self.pending_gear is None:
            return str(self.gear)
        return "{}->{}".format(self.gear, self.pending_gear)

    def start_shift(self, target_gear, speed_kph):
        self.pending_gear = int(target_gear)
        self.shift_direction = 1 if self.pending_gear > self.gear else -1
        self.shift_elapsed_s = 0.0
        self.shift_duration_s = (
            self.upshift_duration_s if self.shift_direction > 0 else self.downshift_duration_s
        )
        self.shift_start_rpm = self.rpm
        if self.shift_direction < 0:
            current_rpm = self.rpm_for_gear(speed_kph, self.gear)
            target_rpm = self.rpm_for_gear(speed_kph, self.pending_gear)
            self.shift_blip = clamp(
                (target_rpm - current_rpm) / 2500.0 * self.blip_gain,
                0.0,
                1.0,
            )
        else:
            self.shift_blip = 0.0

    def update_shift(self, speed_kph, dt, load):
        self.shift_elapsed_s += dt
        progress = clamp(self.shift_elapsed_s / self.shift_duration_s, 0.0, 1.0)
        target_rpm = self.rpm_for_gear(speed_kph, self.pending_gear)

        if self.shift_direction > 0:
            # Upshift: ignition/torque cut while RPM falls into the next ratio.
            self.rpm = self.shift_start_rpm + smoothstep(progress) * (
                target_rpm - self.shift_start_rpm
            )
            self.torque_scale = 0.45
            load *= self.torque_scale
        else:
            # Downshift: interpolate into the lower ratio while the virtual
            # throttle blip controls the audible engine load.
            self.rpm = self.shift_start_rpm + smoothstep(progress) * (
                target_rpm - self.shift_start_rpm
            )
            self.torque_scale = 0.75 + 0.25 * self.shift_blip
            load = max(load, self.torque_scale)

        shift_effect = self.shift_direction * math.sin(math.pi * progress)
        if progress >= 1.0:
            self.gear = self.pending_gear
            self.pending_gear = None
            self.shift_direction = 0
            self.rpm = target_rpm
            self.shift_cooldown_remaining_s = self.shift_cooldown_s
            self.shift_blip = 0.0
            self.torque_scale = 1.0

        return self.rpm, self.gear, load, shift_effect

    def update(self, speed_kph, dt, throttle=0.0, brake=0.0):
        speed_kph = clamp(float(speed_kph), 0.0, self.max_speed_kph)
        throttle = clamp(float(throttle), 0.0, 1.0)
        brake = clamp(float(brake), 0.0, 1.0)
        dt = max(1e-3, float(dt))
        kickdown_requested = (
            throttle > self.kickdown_throttle
            and self.previous_throttle <= self.kickdown_throttle
            and self.gear > 1
            and self.pending_gear is None
        )
        self.previous_throttle = throttle

        acceleration_kph_s = (speed_kph - self.previous_speed_kph) / dt
        self.previous_speed_kph = speed_kph
        load = clamp(0.16 + 0.84 * throttle + max(0.0, acceleration_kph_s) * 0.01, 0.12, 1.0)

        if self.pending_gear is not None:
            return self.update_shift(speed_kph, dt, load)

        self.shift_cooldown_remaining_s = max(
            0.0, self.shift_cooldown_remaining_s - dt
        )
        self.shift_blip = 0.0
        self.torque_scale = 1.0
        if self.shift_cooldown_remaining_s <= 0.0:
            current_gear_rpm = self.rpm_for_gear(speed_kph, self.gear)

            # 1. Full-throttle kickdown.
            if kickdown_requested and self.gear > 1:
                target_gear = self.gear - 1
                if self.can_downshift(speed_kph, target_gear):
                    self.start_shift(target_gear, speed_kph)
                    return self.update_shift(speed_kph, dt, load)

            # 2. Sport downshift under braking, only inside the useful RPM band.
            if self.brake_downshift < brake and self.gear > 1:
                target_gear = self.gear - 1
                target_rpm = self.rpm_for_gear(speed_kph, target_gear)
                if 3500.0 <= target_rpm <= self.soft_limit_rpm:
                    self.start_shift(target_gear, speed_kph)
                    return self.update_shift(speed_kph, dt, load)

            # 3. Automatic upshift.
            if (
                self.gear < len(self.gear_max_speeds_kph)
                and current_gear_rpm >= self.upshift_rpm
            ):
                self.start_shift(self.gear + 1, speed_kph)
                return self.update_shift(speed_kph, dt, load)

            # 4. Low-RPM downshift.
            if self.gear > 1:
                target_gear = self.gear - 1
                if (
                    current_gear_rpm < self.downshift_rpm
                    and self.can_downshift(speed_kph, target_gear)
                ):
                    self.start_shift(target_gear, speed_kph)
                    return self.update_shift(speed_kph, dt, load)

        target_rpm = self.rpm_for_gear(speed_kph, self.gear)
        smoothing_tau = self.rpm_rise_tau_s if target_rpm > self.rpm else self.rpm_fall_tau_s
        alpha = 1.0 if smoothing_tau <= 0.0 else 1.0 - math.exp(-dt / smoothing_tau)
        self.rpm += alpha * (target_rpm - self.rpm)

        return self.rpm, self.gear, load, 0.0


class ProceduralEngine:
    """Generate continuous mono PCM for a configurable combustion-engine style."""

    def __init__(
        self,
        sample_rate,
        volume,
        idle_rpm,
        max_rpm,
        cylinders,
        engine_style,
        ev_low_drone_gain,
        ev_high_layer_gain,
    ):
        self.sample_rate = int(sample_rate)
        self.volume = clamp(float(volume), 0.0, 1.0)
        self.idle_rpm = float(idle_rpm)
        self.max_rpm = max(self.idle_rpm + 1.0, float(max_rpm))
        self.combustion_events_per_rev = max(1.0, float(cylinders) * 0.5)
        self.engine_style = str(engine_style).lower()
        self.ev_low_drone_gain = max(0.0, float(ev_low_drone_gain))
        self.ev_high_layer_gain = max(0.0, float(ev_high_layer_gain))
        self.combustion_phase = 0.0
        self.shaft_phase = 0.0
        self.motor2_phase = 0.0
        self.turbo_phase = 0.0
        self.space1_phase = 0.0
        self.space2_phase = 0.0
        self.space3_phase = 0.0
        self.previous_rpm = 1300.0
        self.smoothed_load = 0.0
        self.noise_state = 0.0
        self.previous_shift_kick = 0.0
        self.gear_engagement_transient = 0.0
        self.gear_thump_phase = 0.0
        self.rng = np.random.RandomState(2026)

    def render(self, frames, rpm, load, shift_kick, signal_gain):
        frames = int(frames)
        chunk_dt = frames / float(self.sample_rate)
        load_alpha = 1.0 - math.exp(-chunk_dt / 0.070)
        self.smoothed_load += load_alpha * (clamp(load, 0.0, 1.0) - self.smoothed_load)
        load = self.smoothed_load
        rpm_line = np.linspace(self.previous_rpm, rpm, frames, endpoint=False)
        self.previous_rpm = rpm

        # A four-stroke V8 has four combustion events per crank revolution.
        combustion_hz = rpm_line / 60.0 * self.combustion_events_per_rev
        shaft_hz = rpm_line / 60.0
        # A compact inline-four uses a quieter, lower turbo layer than the V8 preset.
        if self.engine_style == "ev_spaceship":
            turbo_hz = 460.0 + 0.055 * rpm_line
        elif self.engine_style == "inline4":
            turbo_hz = 330.0 + 0.030 * rpm_line
        else:
            turbo_hz = 380.0 + 0.045 * rpm_line

        combustion_inc = 2.0 * math.pi * combustion_hz / self.sample_rate
        shaft_inc = 2.0 * math.pi * shaft_hz / self.sample_rate
        # Same electrical order with a phase offset: wide stereo without an
        # audible low-frequency beating cycle at constant speed.
        motor2_inc = 2.0 * math.pi * shaft_hz * 2.00 / self.sample_rate
        turbo_inc = 2.0 * math.pi * turbo_hz / self.sample_rate
        space1_hz = 74.0 + 0.010 * rpm_line
        space1_inc = 2.0 * math.pi * space1_hz / self.sample_rate
        space2_inc = 2.0 * math.pi * space1_hz * 1.38 / self.sample_rate
        space3_inc = 2.0 * math.pi * space1_hz * 2.55 / self.sample_rate

        combustion_phase = self.combustion_phase + np.cumsum(combustion_inc)
        shaft_phase = self.shaft_phase + np.cumsum(shaft_inc)
        motor2_phase = self.motor2_phase + np.cumsum(motor2_inc)
        turbo_phase = self.turbo_phase + np.cumsum(turbo_inc)
        space1_phase = self.space1_phase + np.cumsum(space1_inc)
        space2_phase = self.space2_phase + np.cumsum(space2_inc)
        space3_phase = self.space3_phase + np.cumsum(space3_inc)
        self.combustion_phase = float(combustion_phase[-1] % (2.0 * math.pi))
        self.shaft_phase = float(shaft_phase[-1] % (2.0 * math.pi))
        self.motor2_phase = float(motor2_phase[-1] % (2.0 * math.pi))
        self.turbo_phase = float(turbo_phase[-1] % (2.0 * math.pi))
        self.space1_phase = float(space1_phase[-1] % (2.0 * math.pi))
        self.space2_phase = float(space2_phase[-1] % (2.0 * math.pi))
        self.space3_phase = float(space3_phase[-1] % (2.0 * math.pi))

        side_signal = np.zeros(frames, dtype=np.float64)
        if self.engine_style == "ev_spaceship":
            # Original EV sound design: two slightly detuned motor-like layers,
            # inverter harmonics and an airy high-frequency carrier.
            motor_tone = (
                0.48 * np.sin(2.0 * shaft_phase)
                + 0.22 * np.sin(4.0 * shaft_phase + 0.20)
                + 0.10 * np.sin(7.0 * shaft_phase + 0.52)
            )
            detuned_motor = (
                0.20 * np.sin(motor2_phase + 0.65)
                + 0.09 * np.sin(2.0 * motor2_phase + 0.10)
            )
            space_chord = (
                0.28 * np.sin(space1_phase)
                + 0.20 * np.sin(space2_phase + 0.35)
                + 0.13 * np.sin(space3_phase + 0.80)
            )
            drive_mix = 0.32 + 0.68 * load
            exhaust = np.tanh(
                (
                    drive_mix * (motor_tone + detuned_motor)
                    + (0.58 + 0.42 * load) * space_chord
                )
                * (1.02 + 0.24 * load)
            )
            mechanical = (
                0.055 * np.sin(shaft_phase + 0.30)
                + 0.035 * np.sin(5.0 * shaft_phase + 0.10)
            )
            turbo = (0.018 + 0.045 * load) * (
                np.sin(turbo_phase) + 0.28 * np.sin(2.0 * turbo_phase + 0.35)
            )
            # Opposite-polarity side information produces the wide dual-motor
            # image without adding random hiss.
            side_signal = (
                0.22 * np.sin(2.0 * shaft_phase + 0.5 * math.pi)
                + 0.11 * np.sin(4.0 * shaft_phase + 0.5 * math.pi)
                + 0.14 * np.sin(motor2_phase - 0.45)
                + 0.12 * np.sin(space2_phase + 1.15)
                + 0.055 * np.sin(turbo_phase + 0.5 * math.pi)
            ) * 0.72
        elif self.engine_style == "inline4":
            firing_pulse = (
                0.72 * np.sin(combustion_phase)
                + 0.25 * np.sin(2.0 * combustion_phase + 0.20)
                + 0.09 * np.sin(3.0 * combustion_phase + 0.48)
            )
            intake = (
                0.25 * np.sin(shaft_phase + 0.35)
                + 0.19 * np.sin(2.0 * shaft_phase + 0.10)
            )
            exhaust = np.tanh((firing_pulse + intake) * (1.55 + 0.55 * load))
            mechanical = (
                0.08 * np.sin(2.0 * shaft_phase)
                + 0.035 * np.sin(4.0 * shaft_phase + 0.20)
            )
            turbo = (0.006 + 0.016 * load) * np.sin(turbo_phase)
        else:
            bank_wobble = 0.76 + 0.24 * np.sin(2.0 * shaft_phase + 0.35)
            firing_pulse = (
                0.58 * np.sin(combustion_phase)
                + 0.24 * np.sin(2.0 * combustion_phase + 0.18)
                + 0.10 * np.sin(3.0 * combustion_phase + 0.52)
            )
            low_burble = (
                0.46 * np.sin(2.0 * shaft_phase + 0.12)
                + 0.24 * np.sin(shaft_phase + 0.70)
                + 0.13 * np.sin(3.0 * shaft_phase + 0.31)
            )
            exhaust = np.tanh(
                (bank_wobble * firing_pulse + low_burble) * (1.8 + 0.65 * load)
            )
            mechanical = 0.10 * np.sin(4.0 * shaft_phase) + 0.05 * np.sin(6.0 * shaft_phase)
            turbo = (0.010 + 0.030 * load) * np.sin(turbo_phase)

        if self.engine_style == "ev_spaceship":
            # A clean EV layer must not contain combustion/broadband noise.
            white = np.zeros(frames, dtype=np.float64)
            colored = white
        else:
            white = self.rng.normal(0.0, 1.0, frames)
            colored = np.empty(frames, dtype=np.float64)
            state = self.noise_state
            for i, sample in enumerate(white):
                state += 0.12 * (sample - state)
                colored[i] = state
            self.noise_state = float(state)

        rpm_ratio = clamp((rpm - self.idle_rpm) / (self.max_rpm - self.idle_rpm), 0.0, 1.0)
        rpm_gain = 0.34 + 0.66 * rpm_ratio
        if self.engine_style == "ev_spaceship":
            signal = rpm_gain * self.ev_high_layer_gain * (
                0.88 * exhaust + mechanical + turbo
            )
        else:
            signal = rpm_gain * (0.88 * exhaust + mechanical + turbo)
        if self.engine_style == "ev_spaceship":
            # Bring in a low motor drone almost immediately after launch, then
            # taper it at high speed so it does not create a constant top-speed hum.
            motion_presence = clamp(rpm_ratio / 0.07, 0.0, 1.0)
            low_speed_focus = motion_presence * math.pow(max(0.0, 1.0 - rpm_ratio), 0.70)
            signal += self.ev_low_drone_gain * low_speed_focus * (
                0.30 * np.sin(space1_phase)
                + 0.12 * np.sin(space2_phase + 0.22)
            )
        # Keep broadband noise subtle; RPM motion itself carries the shift sound.
        if self.engine_style != "ev_spaceship":
            signal += colored * (0.010 + 0.018 * load)

        # Cut ignition/torque through the middle of an upshift. Downshifts retain
        # most of the engine layer so the rev-match blip remains audible.
        if shift_kick > 0.0:
            signal *= 1.0 - 0.72 * shift_kick
        elif shift_kick < 0.0:
            signal *= 1.0 - 0.12 * abs(shift_kick)

        # Detect completion of either shift and synthesize one short gearbox
        # engagement transient instead of a long broadband whoosh.
        if abs(self.previous_shift_kick) > 0.05 and abs(shift_kick) < 0.01:
            self.gear_engagement_transient = 1.0
            self.gear_thump_phase = 0.0
        self.previous_shift_kick = shift_kick

        sample_index = np.arange(frames, dtype=np.float64)
        engagement_envelope = self.gear_engagement_transient * np.exp(
            -sample_index / (self.sample_rate * 0.045)
        )
        thump_inc = 2.0 * math.pi * 68.0 / self.sample_rate
        thump_phase = self.gear_thump_phase + thump_inc * (sample_index + 1.0)
        engagement_scale = 0.35 if self.engine_style == "ev_spaceship" else 1.0
        gear_thump = engagement_scale * engagement_envelope * (
            0.18 * np.sin(thump_phase)
            + 0.055 * np.sin(2.0 * thump_phase + 0.25)
            + 0.018 * white
        )
        signal += gear_thump
        self.gear_thump_phase = float(thump_phase[-1] % (2.0 * math.pi))
        self.gear_engagement_transient = float(engagement_envelope[-1])

        output_gain = self.volume * clamp(signal_gain, 0.0, 1.0)
        signal *= output_gain
        side_signal *= output_gain
        drive = 0.95 if self.engine_style == "ev_spaceship" else 1.25
        left = np.tanh((signal + side_signal) * drive) * 0.82
        right = np.tanh((signal - side_signal) * drive) * 0.82
        stereo = np.empty(frames * 2, dtype="<i2")
        stereo[0::2] = np.asarray(left * 32767.0, dtype="<i2")
        stereo[1::2] = np.asarray(right * 32767.0, dtype="<i2")
        return stereo.tobytes()


class AudioSink:
    def __init__(self, backend, sample_rate, latency_ms, process_time_ms):
        self.backend = str(backend).lower()
        self.sample_rate = int(sample_rate)
        self.process = None

        if self.backend == "none":
            rospy.logwarn("[engine_sound] audio backend=none; state is generated without audio output")
            return

        executable = shutil.which(self.backend)
        if executable is None:
            raise RuntimeError("audio backend not found: {}".format(self.backend))

        if self.backend == "paplay":
            command = [
                executable, "--raw", "--format=s16le",
                "--rate={}".format(self.sample_rate), "--channels=2",
                "--latency-msec={:g}".format(float(latency_ms)),
                "--process-time-msec={:g}".format(float(process_time_ms)),
            ]
        elif self.backend == "aplay":
            command = [
                executable, "-q", "-t", "raw", "-f", "S16_LE",
                "-r", str(self.sample_rate), "-c", "2",
                "-B", str(max(1000, int(float(latency_ms) * 1000.0))),
                "-F", str(max(1000, int(float(process_time_ms) * 1000.0))),
            ]
        else:
            raise RuntimeError("unsupported audio backend: {}".format(self.backend))

        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, bufsize=0)
        rospy.loginfo("[engine_sound] audio output started: %s", " ".join(command))

    def write(self, pcm):
        if self.process is None:
            return
        if self.process.poll() is not None:
            raise RuntimeError("audio backend terminated with code {}".format(self.process.returncode))
        self.process.stdin.write(pcm)

    def close(self):
        if self.process is None:
            return
        try:
            if self.process.stdin:
                self.process.stdin.close()
        except (BrokenPipeError, OSError):
            pass
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
        self.process = None


class MoraiEngineSoundNode:
    def __init__(self):
        self.ego_topic = rospy.get_param("~ego_topic", "/morai/ego_topic")
        self.velocity_scale_to_kph = float(rospy.get_param("~velocity_scale_to_kph", 1.0))
        self.sample_rate = int(rospy.get_param("~sample_rate", 48000))
        self.chunk_ms = float(rospy.get_param("~chunk_ms", 10.0))
        self.volume = float(rospy.get_param("~volume", 0.55))
        self.engine_style = rospy.get_param("~engine_style", "ev_spaceship")
        self.engine_cylinders = int(rospy.get_param("~engine_cylinders", 4))
        self.ev_low_drone_gain = float(rospy.get_param("~ev_low_drone_gain", 2.0))
        self.ev_high_layer_gain = float(rospy.get_param("~ev_high_layer_gain", 0.70))
        self.backend = rospy.get_param("~audio_backend", "paplay")
        self.audio_latency_ms = float(rospy.get_param("~audio_latency_ms", 20.0))
        self.audio_process_time_ms = float(rospy.get_param("~audio_process_time_ms", 5.0))
        self.state_publish_hz = max(1.0, float(rospy.get_param("~state_publish_hz", 50.0)))
        self.message_timeout_s = float(rospy.get_param("~message_timeout_s", 0.5))
        self.frames_per_chunk = max(128, int(self.sample_rate * self.chunk_ms / 1000.0))

        self.lock = threading.Lock()
        self.speed_kph = 0.0
        self.throttle = 0.0
        self.brake = 0.0
        self.last_message_time = None
        self.signal_gain = 0.0

        self.powertrain = VirtualPowertrain()
        self.synth = ProceduralEngine(
            self.sample_rate,
            self.volume,
            self.powertrain.idle_rpm,
            self.powertrain.max_rpm,
            self.engine_cylinders,
            self.engine_style,
            self.ev_low_drone_gain,
            self.ev_high_layer_gain,
        )
        self.audio = AudioSink(
            self.backend,
            self.sample_rate,
            self.audio_latency_ms,
            self.audio_process_time_ms,
        )

        self.rpm_pub = rospy.Publisher("~rpm", Float64, queue_size=1)
        self.gear_pub = rospy.Publisher("~gear", Int16, queue_size=1)
        self.shift_state_pub = rospy.Publisher("~shift_state", String, queue_size=1)
        self.blip_pub = rospy.Publisher("~rpm_blip", Float64, queue_size=1)
        self.torque_scale_pub = rospy.Publisher("~torque_scale", Float64, queue_size=1)
        self.speed_pub = rospy.Publisher("~speed_kph", Float64, queue_size=1)
        self.ego_sub = rospy.Subscriber(
            self.ego_topic, EgoVehicleStatus, self.ego_callback, queue_size=1
        )
        rospy.on_shutdown(self.audio.close)
        rospy.loginfo(
            "[engine_sound] listening=%s scale_to_kph=%.3f volume=%.2f backend=%s",
            self.ego_topic, self.velocity_scale_to_kph, self.volume, self.backend,
        )

    def ego_callback(self, msg):
        raw_speed = math.hypot(msg.velocity.x, msg.velocity.y)
        with self.lock:
            self.speed_kph = max(0.0, raw_speed * self.velocity_scale_to_kph)
            self.throttle = clamp(msg.accel, 0.0, 1.0)
            self.brake = clamp(msg.brake, 0.0, 1.0)
            self.last_message_time = time.monotonic()

    def run(self):
        chunk_dt = self.frames_per_chunk / float(self.sample_rate)
        next_status_time = time.monotonic()

        while not rospy.is_shutdown():
            loop_start = time.monotonic()
            with self.lock:
                speed_kph = self.speed_kph
                throttle = self.throttle
                brake = self.brake
                last_message_time = self.last_message_time

            message_fresh = (
                last_message_time is not None
                and loop_start - last_message_time <= self.message_timeout_s
            )
            target_gain = 1.0 if message_fresh else 0.0
            gain_alpha = 1.0 - math.exp(-chunk_dt / 0.12)
            self.signal_gain += gain_alpha * (target_gain - self.signal_gain)

            rpm, gear, load, shift_kick = self.powertrain.update(
                speed_kph, chunk_dt, throttle, brake
            )
            pcm = self.synth.render(
                self.frames_per_chunk, rpm, load, shift_kick, self.signal_gain
            )
            try:
                self.audio.write(pcm)
            except (BrokenPipeError, OSError, RuntimeError) as error:
                rospy.logerr("[engine_sound] audio output failed: %s", error)
                self.audio.close()
                return

            now = time.monotonic()
            if now >= next_status_time:
                self.rpm_pub.publish(Float64(data=rpm))
                self.gear_pub.publish(Int16(data=gear))
                shift_label = self.powertrain.shift_label()
                self.shift_state_pub.publish(String(data=shift_label))
                self.blip_pub.publish(Float64(data=self.powertrain.shift_blip))
                self.torque_scale_pub.publish(
                    Float64(data=self.powertrain.torque_scale)
                )
                self.speed_pub.publish(Float64(data=speed_kph))
                rospy.loginfo(
                    "[engine_sound] speed=%.1f kph gear=%s rpm=%.0f",
                    speed_kph, shift_label, rpm,
                )
                next_status_time = now + 1.0 / self.state_publish_hz

            if self.backend == "none":
                elapsed = time.monotonic() - loop_start
                if elapsed < chunk_dt:
                    time.sleep(chunk_dt - elapsed)


def main():
    rospy.init_node("morai_engine_sound", anonymous=False)
    try:
        node = MoraiEngineSoundNode()
        node.run()
    except (RuntimeError, OSError) as error:
        rospy.logfatal("[engine_sound] startup failed: %s", error)


if __name__ == "__main__":
    main()
