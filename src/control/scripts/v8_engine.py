#!/usr/bin/env python3

import math
import os
import threading
import time
import wave

import numpy as np
import rospy
from morai_msgs.msg import EgoVehicleStatus
from std_msgs.msg import Bool, Float64, Int16, String

from morai_engine_sound_ev import AudioSink, VirtualPowertrain, clamp


class CompressedICEPowertrain(VirtualPowertrain):
    """Six-speed ICE model compressed into the simulator's 0-100 km/h range."""

    def rpm_for_gear(self, speed_kph, gear):
        speed_kph = clamp(float(speed_kph), 0.0, self.max_speed_kph)
        index = int(clamp(
            int(gear) - 1, 0, len(self.gear_max_speeds_kph) - 1
        ))
        gear_redline_speed_kph = self.gear_max_speeds_kph[index]
        coupled_rpm = (
            self.redline_rpm * speed_kph / gear_redline_speed_kph
        )
        return clamp(
            max(self.idle_rpm, coupled_rpm),
            self.idle_rpm,
            self.max_rpm,
        )


class DampedCombResonator:
    """Small recursive exhaust-pipe resonator with damping in the feedback path."""

    def __init__(self, sample_rate, delay_s, feedback, damping):
        delay_samples = max(2, int(float(sample_rate) * float(delay_s)))
        self.buffer = np.zeros(delay_samples, dtype=np.float64)
        self.index = 0
        self.feedback = clamp(float(feedback), 0.0, 0.92)
        self.damping = clamp(float(damping), 0.01, 1.0)
        self.filtered_feedback = 0.0

    def process(self, signal):
        output = np.empty_like(signal)
        index = self.index
        filtered = self.filtered_feedback
        buffer = self.buffer
        for sample_index, value in enumerate(signal):
            delayed = buffer[index]
            filtered += self.damping * (delayed - filtered)
            output[sample_index] = value + filtered
            buffer[index] = value + self.feedback * filtered
            index += 1
            if index >= len(buffer):
                index = 0
        self.index = index
        self.filtered_feedback = float(filtered)
        return output


def _decode_pcm_bytes(raw, channels, sample_width, path):
    if channels not in (1, 2):
        raise RuntimeError("unsupported WAV channel count: {}".format(channels))
    if sample_width == 2:
        audio = np.frombuffer(raw, dtype="<i2").astype(np.float64) / 32768.0
    elif sample_width == 3:
        packed = np.frombuffer(raw, dtype=np.uint8).reshape(-1, 3)
        values = (
            packed[:, 0].astype(np.int32)
            | (packed[:, 1].astype(np.int32) << 8)
            | (packed[:, 2].astype(np.int32) << 16)
        )
        values = np.where(values & 0x800000, values - 0x1000000, values)
        audio = values.astype(np.float64) / 8388608.0
    else:
        raise RuntimeError(
            "only 16/24-bit PCM WAV is supported: {}".format(path)
        )
    audio = audio.reshape(-1, channels)
    if channels == 1:
        audio = np.repeat(audio, 2, axis=1)
    audio -= np.mean(audio, axis=0, keepdims=True)
    return audio.astype(np.float32)


def load_pcm_wav(path):
    """Load mono/stereo 16- or 24-bit PCM WAV as normalized stereo."""
    with wave.open(path, "rb") as wav_file:
        sample_rate = int(wav_file.getframerate())
        channels = int(wav_file.getnchannels())
        sample_width = int(wav_file.getsampwidth())
        raw = wav_file.readframes(wav_file.getnframes())
    return _decode_pcm_bytes(raw, channels, sample_width, path), sample_rate


def load_pcm_wav_segment(path, start_s, end_s):
    """Decode only one time range, avoiding a full long-recording allocation."""
    with wave.open(path, "rb") as wav_file:
        sample_rate = int(wav_file.getframerate())
        channels = int(wav_file.getnchannels())
        sample_width = int(wav_file.getsampwidth())
        total_frames = int(wav_file.getnframes())
        start_frame = int(clamp(float(start_s) * sample_rate, 0, total_frames - 2))
        end_frame = int(clamp(
            float(end_s) * sample_rate, start_frame + 2, total_frames
        ))
        wav_file.setpos(start_frame)
        raw = wav_file.readframes(end_frame - start_frame)
    return _decode_pcm_bytes(raw, channels, sample_width, path), sample_rate


class StartupIdleSample:
    """Play the starter once, then hold the recorded idle while stationary."""

    def __init__(self, path, output_sample_rate, loop_start_s, loop_end_s, gain):
        self.audio, self.sample_rate = load_pcm_wav(path)
        if self.sample_rate != int(output_sample_rate):
            raise RuntimeError("startup WAV sample rate must match audio output")
        self.loop_start = int(loop_start_s * self.sample_rate)
        self.loop_end = int(loop_end_s * self.sample_rate)
        self.loop_start = int(clamp(self.loop_start, 0, len(self.audio) - 2))
        self.loop_end = int(clamp(
            self.loop_end, self.loop_start + 2, len(self.audio) - 2
        ))
        self.crossfade_frames = min(
            int(0.18 * self.sample_rate),
            max(64, (self.loop_end - self.loop_start) // 4),
        )
        self.position = 0.0
        self.gain = max(0.0, float(gain))
        self.drive_mix = 0.0
        self.sample_mix = 1.0
        self.state = "startup"
        self.stop_hold_s = 0.0

    def _read(self, position):
        position = clamp(position, 0.0, len(self.audio) - 2.0)
        index = int(position)
        fraction = position - index
        return self.audio[index] * (1.0 - fraction) + self.audio[index + 1] * fraction

    def render(self, frames, speed_kph, acceleration_kph_s):
        frames = int(frames)
        output = np.zeros((frames, 2), dtype=np.float64)
        moving = speed_kph > 1.2 or acceleration_kph_s > 0.8
        near_stop = speed_kph < 0.6
        chunk_dt = frames / float(self.sample_rate)

        if moving and self.state in ("startup", "idle", "to_idle"):
            self.state = "to_drive"
            self.stop_hold_s = 0.0
        elif self.state == "driving":
            self.stop_hold_s = self.stop_hold_s + chunk_dt if near_stop else 0.0
            if self.stop_hold_s >= 0.25:
                self.state = "to_idle"
                self.position = float(self.loop_start)
                self.stop_hold_s = 0.0

        fade_step = 1.0 / max(1, int(0.18 * self.sample_rate))
        for index in range(int(frames)):
            target_sample_mix = 0.0 if self.state in ("to_drive", "driving") else 1.0
            if self.sample_mix < target_sample_mix:
                self.sample_mix = min(target_sample_mix, self.sample_mix + fade_step)
            elif self.sample_mix > target_sample_mix:
                self.sample_mix = max(target_sample_mix, self.sample_mix - fade_step)
            self.drive_mix = 1.0 - self.sample_mix

            if self.state != "driving":
                sample = self._read(self.position)
                fade_start = self.loop_end - self.crossfade_frames
                if self.position >= fade_start:
                    fade = (self.position - fade_start) / self.crossfade_frames
                    loop_position = self.loop_start + (self.position - fade_start)
                    sample = sample * (1.0 - fade) + self._read(loop_position) * fade
                output[index] = sample * self.sample_mix
                self.position += 1.0
                if self.position >= self.loop_end:
                    self.position = float(self.loop_start + self.crossfade_frames)

            if self.state == "to_drive" and self.sample_mix <= 0.0:
                self.state = "driving"
            elif self.state == "to_idle" and self.sample_mix >= 1.0:
                self.state = "idle"

        return output * self.gain


class M3GearGranularSample:
    """Map extracted M3 pull/overrun regions to virtual gear and engine RPM."""

    def __init__(
        self, path, output_sample_rate, gear_segments_s, overrun_segment_s,
        idle_rpm, upshift_rpm, gain,
    ):
        self.audio, self.sample_rate = load_pcm_wav(path)
        if self.sample_rate != int(output_sample_rate):
            raise RuntimeError("M3 WAV sample rate must match audio output")
        self.output_sample_rate = int(output_sample_rate)
        self.gear_segments = [
            (int(a * self.sample_rate), int(b * self.sample_rate))
            for a, b in gear_segments_s
        ]
        self.overrun_segment = (
            int(overrun_segment_s[0] * self.sample_rate),
            int(overrun_segment_s[1] * self.sample_rate),
        )
        self.idle_rpm = float(idle_rpm)
        self.upshift_rpm = max(self.idle_rpm + 1.0, float(upshift_rpm))
        self.gain = max(0.0, float(gain))
        self.grain_frames = int(0.30 * self.output_sample_rate)
        self.grain_count = 4
        self.grain_hop = self.grain_frames // self.grain_count
        self.jitter_frames = int(0.010 * self.sample_rate)
        self.rng = np.random.RandomState(3030)
        self.grains = []
        self.next_grain_in = 0
        self.last_center = None

    def _segment_for_state(self, gear, throttle, acceleration_kph_s):
        overrun = throttle < 0.10 and acceleration_kph_s < -0.5
        if overrun:
            return self.overrun_segment, True
        index = int(clamp(int(gear) - 1, 0, len(self.gear_segments) - 1))
        return self.gear_segments[index], False

    def _spawn_grain(self, age, center, segment):
        start, end = segment
        jitter = self.rng.uniform(-self.jitter_frames, self.jitter_frames)
        source_start = center + jitter - 0.5 * self.grain_frames
        source_start = clamp(
            source_start, start, max(start, end - self.grain_frames - 2)
        )
        self.grains.append({"age": int(age), "source_start": float(source_start)})

    def _read_linear(self, positions):
        positions = np.clip(positions, 0, len(self.audio) - 2)
        index = np.floor(positions).astype(np.int32)
        fraction = (positions - index).reshape(-1, 1)
        return self.audio[index] * (1.0 - fraction) + self.audio[index + 1] * fraction

    def render(self, frames, rpm, gear, throttle, acceleration_kph_s):
        frames = int(frames)
        segment, overrun = self._segment_for_state(
            gear, throttle, acceleration_kph_s
        )
        rpm_ratio = clamp(
            (float(rpm) - self.idle_rpm) / (self.upshift_rpm - self.idle_rpm),
            0.0,
            1.0,
        )
        if overrun:
            center = segment[0] + (1.0 - rpm_ratio) * (segment[1] - segment[0])
        else:
            center = segment[0] + rpm_ratio * (segment[1] - segment[0])
        margin = 0.5 * self.grain_frames + 2
        center = clamp(center, segment[0] + margin, segment[1] - margin)
        if self.last_center is None:
            self.last_center = center
            for grain_index in range(self.grain_count):
                self._spawn_grain(
                    grain_index * self.grain_hop, center, segment
                )
            self.next_grain_in = self.grain_hop
        else:
            # Limit source-window movement to avoid clicks from quantized speed input.
            maximum_move = 0.040 * self.sample_rate
            center = self.last_center + clamp(
                center - self.last_center, -maximum_move, maximum_move
            )
            self.last_center = center

        while self.next_grain_in < frames:
            self._spawn_grain(-self.next_grain_in, center, segment)
            self.next_grain_in += self.grain_hop
        self.next_grain_in -= frames

        output = np.zeros((frames, 2), dtype=np.float64)
        weight = np.zeros(frames, dtype=np.float64)
        offsets = np.arange(frames, dtype=np.int32)
        alive = []
        for grain in self.grains:
            ages = grain["age"] + offsets
            active = (ages >= 0) & (ages < self.grain_frames)
            if np.any(active):
                active_ages = ages[active].astype(np.float64)
                phase = active_ages / (self.grain_frames - 1.0)
                envelope = 0.5 - 0.5 * np.cos(2.0 * math.pi * phase)
                positions = grain["source_start"] + active_ages
                output[active] += self._read_linear(positions) * envelope.reshape(-1, 1)
                weight[active] += envelope
            grain["age"] += frames
            if grain["age"] < self.grain_frames:
                alive.append(grain)
        self.grains = alive
        output /= np.maximum(weight, 1e-5).reshape(-1, 1)
        drive_gain = 0.72 if overrun else 0.55 + 0.45 * clamp(throttle, 0.0, 1.0)
        return output * self.gain * drive_gain


class M3TransientSample:
    """Play untouched M3 pull/overrun regions only in matching vehicle states."""

    def __init__(
        self, path, output_sample_rate, gear_segments_s, overrun_segment_s,
        gear_start_rpm, upshift_rpm, gain,
    ):
        self.gear_audio = []
        self.sample_rate = int(output_sample_rate)
        for start_s, end_s in gear_segments_s:
            audio, sample_rate = load_pcm_wav_segment(path, start_s, end_s)
            if sample_rate != self.sample_rate:
                raise RuntimeError("M3 WAV sample rate must match audio output")
            self.gear_audio.append(audio)
        self.overrun_audio, sample_rate = load_pcm_wav_segment(
            path, overrun_segment_s[0], overrun_segment_s[1]
        )
        if sample_rate != self.sample_rate:
            raise RuntimeError("M3 WAV sample rate must match audio output")
        self.gear_start_rpm = [float(value) for value in gear_start_rpm]
        self.upshift_rpm = float(upshift_rpm)
        self.gain = max(0.0, float(gain))
        self.mode = "off"
        self.gear = 0
        self.position = 0.0
        self.playback_rate = 1.0
        self.activity_gain = 0.0
        self.current_audio = None

    def _gear_index(self, gear):
        return int(clamp(int(gear) - 1, 0, len(self.gear_audio) - 1))

    @staticmethod
    def _read_linear(audio, positions):
        positions = np.clip(positions, 0, len(audio) - 2)
        index = np.floor(positions).astype(np.int32)
        fraction = (positions - index).reshape(-1, 1)
        return audio[index] * (1.0 - fraction) + audio[index + 1] * fraction

    def render(
        self, frames, rpm, gear, throttle, acceleration_kph_s, shift_kick,
    ):
        frames = int(frames)
        accelerating = throttle > 0.08 and acceleration_kph_s > 0.5
        overrunning = (
            throttle < 0.08 and acceleration_kph_s < -0.8 and rpm > 1800.0
        )
        requested_mode = "accel" if accelerating else "overrun" if overrunning else "off"

        if requested_mode == "accel":
            index = self._gear_index(gear)
            if self.mode != "accel" or self.gear != int(gear):
                self.mode = "accel"
                self.gear = int(gear)
                self.current_audio = self.gear_audio[index]
                self.position = 0.0
                self.playback_rate = 1.0
        elif requested_mode == "overrun":
            if self.mode != "overrun":
                self.mode = "overrun"
                self.current_audio = self.overrun_audio
                self.position = 0.0
                self.playback_rate = 1.0
        else:
            self.mode = "off"

        if self.mode == "accel":
            index = self._gear_index(gear)
            segment_length = len(self.current_audio)
            start_rpm = self.gear_start_rpm[
                min(index, len(self.gear_start_rpm) - 1)
            ]
            rpm_ratio = clamp(
                (float(rpm) - start_rpm)
                / max(1.0, self.upshift_rpm - start_rpm),
                0.0,
                1.0,
            )
            target_position = rpm_ratio * segment_length
            error_s = (target_position - self.position) / self.sample_rate
            target_rate = clamp(1.0 + 0.65 * error_s, 0.68, 1.45)
            self.playback_rate += 0.12 * (target_rate - self.playback_rate)
        elif self.mode == "overrun":
            segment_length = len(self.current_audio)
            self.playback_rate += 0.08 * (1.0 - self.playback_rate)
        else:
            segment_length = len(self.current_audio) if self.current_audio is not None else 0

        target_gain = 1.0 if self.mode != "off" and abs(shift_kick) < 0.20 else 0.0
        gain_step = 1.0 / max(1, int(0.055 * self.sample_rate))
        output = np.zeros((frames, 2), dtype=np.float64)
        if self.current_audio is not None and (self.mode != "off" or self.activity_gain > 0.0):
            positions = self.position + np.arange(frames) * self.playback_rate
            valid = positions < segment_length - 2
            if np.any(valid):
                output[valid] = self._read_linear(
                    self.current_audio, positions[valid]
                )
            if not np.all(valid):
                target_gain = 0.0
            self.position += frames * self.playback_rate

        for index in range(frames):
            if self.activity_gain < target_gain:
                self.activity_gain = min(target_gain, self.activity_gain + gain_step)
            elif self.activity_gain > target_gain:
                self.activity_gain = max(target_gain, self.activity_gain - gain_step)
            output[index] *= self.activity_gain

        return output * self.gain


class SingleGearLoopSample:
    """Pitch one steady high-RPM recording from the virtual engine RPM.

    The source cursor never seeks when speed or gear changes.  Gear changes are
    therefore heard as a continuous pitch drop/rise instead of as a restarted
    clip.  A source-domain crossfade hides the only discontinuity: the loop
    boundary.
    """

    def __init__(
        self, path, output_sample_rate, reference_rpm, loop_start_s,
        loop_end_s, crossfade_s, gain, minimum_rate,
    ):
        self.audio, self.source_sample_rate = load_pcm_wav(path)
        self.output_sample_rate = int(output_sample_rate)
        self.reference_rpm = max(1.0, float(reference_rpm))
        self.gain = max(0.0, float(gain))
        self.minimum_rate = clamp(float(minimum_rate), 0.10, 1.0)

        duration_s = len(self.audio) / float(self.source_sample_rate)
        end_s = duration_s if float(loop_end_s) <= 0.0 else float(loop_end_s)
        self.loop_start = int(clamp(
            float(loop_start_s) * self.source_sample_rate,
            0,
            len(self.audio) - 4,
        ))
        self.loop_end = int(clamp(
            end_s * self.source_sample_rate,
            self.loop_start + 4,
            len(self.audio) - 2,
        ))
        loop_frames = self.loop_end - self.loop_start
        self.crossfade_frames = int(clamp(
            float(crossfade_s) * self.source_sample_rate,
            64,
            max(64, loop_frames // 3),
        ))
        self.loop_period = float(loop_frames - self.crossfade_frames)
        self.crossfade_start = float(loop_frames - 2 * self.crossfade_frames)
        if self.crossfade_start <= 1.0:
            raise RuntimeError("gear-loop WAV range is too short for crossfade")

        self.phase = 0.0
        self.playback_rate = self.minimum_rate
        self.load_gain = 0.5
        self.previous_shift_kick = 0.0
        self.thump_level = 0.0
        self.thump_phase = 0.0

    def _read_linear(self, positions):
        positions = np.clip(positions, 0.0, len(self.audio) - 2.0)
        index = np.floor(positions).astype(np.int32)
        fraction = (positions - index).reshape(-1, 1)
        return (
            self.audio[index] * (1.0 - fraction)
            + self.audio[index + 1] * fraction
        )

    def _read_loop(self, local_positions):
        main_positions = self.loop_start + self.crossfade_frames + local_positions
        output = self._read_linear(main_positions)
        crossfade = local_positions >= self.crossfade_start
        if np.any(crossfade):
            fade = (
                (local_positions[crossfade] - self.crossfade_start)
                / self.crossfade_frames
            )
            # Equal-power fades keep the steady exhaust level from sagging at
            # the seam and are less audible than a hard modulo wrap.
            fade_in = np.sin(0.5 * math.pi * fade)
            fade_out = np.cos(0.5 * math.pi * fade)
            beginning_positions = (
                self.loop_start
                + local_positions[crossfade]
                - self.crossfade_start
            )
            output[crossfade] = (
                output[crossfade] * fade_out.reshape(-1, 1)
                + self._read_linear(beginning_positions) * fade_in.reshape(-1, 1)
            )
        return output

    def render(self, frames, rpm, load, shift_kick, signal_gain):
        frames = int(frames)
        chunk_dt = frames / float(self.output_sample_rate)
        target_rate = clamp(
            float(rpm) / self.reference_rpm,
            self.minimum_rate,
            1.06,
        )
        rate_tau = 0.018 if target_rate > self.playback_rate else 0.028
        rate_alpha = 1.0 - math.exp(-chunk_dt / rate_tau)
        next_rate = self.playback_rate + rate_alpha * (
            target_rate - self.playback_rate
        )
        rate_curve = np.linspace(
            self.playback_rate, next_rate, frames, dtype=np.float64
        )
        source_steps = rate_curve * (
            self.source_sample_rate / float(self.output_sample_rate)
        )
        source_offsets = np.cumsum(source_steps) - source_steps[0]
        local_positions = np.mod(
            self.phase + source_offsets, self.loop_period
        )
        output = self._read_loop(local_positions).astype(np.float64)
        self.phase = float(
            (self.phase + np.sum(source_steps)) % self.loop_period
        )
        self.playback_rate = float(next_rate)

        load_alpha = 1.0 - math.exp(-chunk_dt / 0.055)
        target_load_gain = 0.54 + 0.46 * clamp(float(load), 0.0, 1.0)
        self.load_gain += load_alpha * (target_load_gain - self.load_gain)

        # The powertrain supplies a half-sine shift pulse.  Positive means an
        # upshift ignition cut; negative means a downshift/rev-match blip.
        if shift_kick > 0.0:
            shift_gain = 1.0 - 0.78 * clamp(shift_kick, 0.0, 1.0)
        elif shift_kick < 0.0:
            shift_gain = 1.0 + 0.10 * clamp(-shift_kick, 0.0, 1.0)
        else:
            shift_gain = 1.0

        shift_finished = (
            abs(self.previous_shift_kick) > 0.08 and abs(shift_kick) <= 0.001
        )
        if shift_finished:
            self.thump_level = 1.0 if self.previous_shift_kick > 0.0 else 0.65
        self.previous_shift_kick = float(shift_kick)

        sample_index = np.arange(frames, dtype=np.float64)
        thump_envelope = self.thump_level * np.exp(
            -sample_index / (self.output_sample_rate * 0.040)
        )
        thump_phase = self.thump_phase + (
            2.0 * math.pi * 64.0 / self.output_sample_rate
        ) * (sample_index + 1.0)
        thump = thump_envelope * (
            0.075 * np.sin(thump_phase)
            + 0.025 * np.sin(2.0 * thump_phase + 0.3)
        )
        self.thump_phase = float(thump_phase[-1] % (2.0 * math.pi))
        self.thump_level = float(thump_envelope[-1])

        output *= self.gain * self.load_gain * shift_gain
        output += thump.reshape(-1, 1)
        output *= clamp(float(signal_gain), 0.0, 1.0)
        return output


class ProceduralV8Engine:
    """Real-time procedural four-stroke V8 sound generator."""

    def __init__(self, sample_rate, volume, idle_rpm, max_rpm, cylinders):
        self.sample_rate = int(sample_rate)
        self.volume = clamp(float(volume), 0.0, 1.0)
        self.idle_rpm = float(idle_rpm)
        self.max_rpm = max(self.idle_rpm + 1.0, float(max_rpm))
        self.combustion_events_per_rev = max(1.0, float(cylinders) * 0.5)

        self.enable_crackle = bool(rospy.get_param("~enable_crackle", True))
        self.crackle_min_rpm = float(rospy.get_param("~crackle_min_rpm", 3500.0))
        self.crackle_min_speed_kph = float(
            rospy.get_param("~crackle_min_speed_kph", 15.0)
        )
        self.crackle_gain = max(0.0, float(rospy.get_param("~crackle_gain", 0.45)))
        self.crackle_probability_gain = max(
            0.0, float(rospy.get_param("~crackle_probability_gain", 1.0))
        )
        self.enable_limiter = bool(rospy.get_param("~enable_limiter", True))
        self.limiter_rpm = float(rospy.get_param("~limiter_rpm", 6900.0))
        self.limiter_hz = max(1.0, float(rospy.get_param("~limiter_hz", 28.0)))

        # Cross-plane V8 bank sequence for a common 1-8-7-2-6-5-4-3 order.
        # Uneven events in each bank create the characteristic V8 burble while
        # the combined engine still fires evenly every 90 crank degrees.
        self.firing_bank_pattern = (0, 1, 0, 1, 1, 0, 1, 0)
        self.firing_event_index = 0
        self.firing_phase = 0.0
        self.firing_interval_scale = 1.0
        self.cylinder_gain = (1.02, 0.97, 1.04, 0.98, 1.01, 0.96, 1.03, 0.99)
        self.fast_pulse_state = np.zeros(2, dtype=np.float64)
        self.slow_pulse_state = np.zeros(2, dtype=np.float64)
        self.bank_left_resonator = DampedCombResonator(
            self.sample_rate, 0.0068, 0.58, 0.30
        )
        self.bank_right_resonator = DampedCombResonator(
            self.sample_rate, 0.0074, 0.56, 0.30
        )
        self.common_resonator_short = DampedCombResonator(
            self.sample_rate, 0.0032, 0.34, 0.42
        )
        self.common_resonator_long = DampedCombResonator(
            self.sample_rate, 0.0090, 0.30, 0.24
        )
        self.dc_previous_input = np.zeros(2, dtype=np.float64)
        self.dc_previous_output = np.zeros(2, dtype=np.float64)
        self.shaft_phase = 0.0
        self.limiter_phase = 0.0
        self.thump_phase = 0.0
        self.previous_rpm = self.idle_rpm
        self.previous_throttle = 0.0
        self.previous_shift_kick = 0.0
        self.smoothed_load = 0.16
        self.noise_state = 0.0
        self.gear_thump_level = 0.0
        self.lift_off_timer_s = 0.0
        self.exhaust_temp = 350.0
        self.crackle_intensity = 0.0
        self.limiter_active = False
        self.pop_events = []
        self.rng = np.random.RandomState(2026)

    def _colored_noise(self, frames):
        white = self.rng.uniform(-1.0, 1.0, int(frames))
        colored = np.empty(int(frames), dtype=np.float64)
        state = self.noise_state
        for index, value in enumerate(white):
            state = 0.92 * state + 0.08 * value
            colored[index] = state
        self.noise_state = float(state)
        return white, colored

    def _generate_firing_pulses(self, rpm_line, load, white):
        frames = len(rpm_line)
        impulses = np.zeros((frames, 2), dtype=np.float64)
        firing_hz = rpm_line / 60.0 * self.combustion_events_per_rev
        phase = self.firing_phase
        event_index = self.firing_event_index
        interval_scale = self.firing_interval_scale
        variation = 0.018 + 0.025 * (1.0 - load)

        for index in range(frames):
            phase += firing_hz[index] / (self.sample_rate * interval_scale)
            if phase >= 1.0:
                phase -= 1.0
                bank = self.firing_bank_pattern[event_index % 8]
                cylinder_scale = self.cylinder_gain[event_index % 8]
                random_scale = 1.0 + self.rng.uniform(-variation, variation)
                impulses[index, bank] = (
                    (0.30 + 0.92 * load) * cylinder_scale * random_scale
                )
                event_index += 1
                rpm_ratio = clamp(
                    (rpm_line[index] - self.idle_rpm)
                    / (self.max_rpm - self.idle_rpm),
                    0.0,
                    1.0,
                )
                timing_sigma = (
                    0.0025 + 0.0075 * (1.0 - load) * (1.0 - rpm_ratio)
                )
                interval_scale = clamp(
                    1.0 + self.rng.normal(0.0, timing_sigma), 0.975, 1.025
                )

        self.firing_phase = float(phase)
        self.firing_event_index = event_index
        self.firing_interval_scale = float(interval_scale)

        fast_decay = math.exp(-1.0 / (self.sample_rate * 0.0017))
        slow_decay = math.exp(-1.0 / (self.sample_rate * 0.0065))
        pulses = np.empty_like(impulses)
        fast = self.fast_pulse_state.copy()
        slow = self.slow_pulse_state.copy()
        for index in range(frames):
            fast = fast_decay * fast + impulses[index]
            slow = slow_decay * slow + impulses[index]
            valve_pulse = fast - 0.30 * slow
            pulse_noise = white[index] * np.abs(fast) * (0.018 + 0.045 * load)
            pulses[index] = valve_pulse + pulse_noise
        self.fast_pulse_state = fast
        self.slow_pulse_state = slow
        return pulses

    def _remove_dc(self, stereo):
        output = np.empty_like(stereo)
        previous_input = self.dc_previous_input.copy()
        previous_output = self.dc_previous_output.copy()
        for index in range(len(stereo)):
            current = stereo[index]
            filtered = current - previous_input + 0.995 * previous_output
            output[index] = filtered
            previous_input = current
            previous_output = filtered
        self.dc_previous_input = previous_input
        self.dc_previous_output = previous_output
        return output

    def _update_crackle_state(
        self, dt, rpm, rpm_ratio, speed_kph, gear, throttle, acceleration_kph_s
    ):
        self.exhaust_temp += throttle * rpm_ratio * 30.0 * dt
        self.exhaust_temp -= 10.0 * dt
        self.exhaust_temp = clamp(self.exhaust_temp, 250.0, 800.0)

        lifted = self.previous_throttle > 0.25 and throttle < 0.08
        if lifted and rpm > self.crackle_min_rpm:
            self.lift_off_timer_s = 0.45
        else:
            self.lift_off_timer_s = max(0.0, self.lift_off_timer_s - dt)

        eligible = (
            self.enable_crackle
            and self.lift_off_timer_s > 0.0
            and rpm > self.crackle_min_rpm
            and speed_kph > self.crackle_min_speed_kph
            and gear >= 2
            and throttle < 0.08
            and acceleration_kph_s < -1.0
        )
        if eligible:
            rpm_factor = clamp((rpm - self.crackle_min_rpm) / 2500.0, 0.0, 1.0)
            drop_factor = clamp((self.previous_throttle - throttle) / 0.70, 0.0, 1.0)
            temp_factor = clamp((self.exhaust_temp - 420.0) / 280.0, 0.0, 1.0)
            self.crackle_intensity = clamp(
                0.45 * rpm_factor + 0.35 * drop_factor + 0.20 * temp_factor,
                0.0,
                1.0,
            )
        else:
            self.crackle_intensity = 0.0

        self.previous_throttle = throttle

    def _maybe_start_pop(self, dt):
        if self.crackle_intensity <= 0.0 or len(self.pop_events) >= 3:
            return
        probability = (
            dt * (4.0 + 12.0 * self.crackle_intensity)
            * self.crackle_probability_gain
        )
        if self.rng.random_sample() >= clamp(probability, 0.0, 0.85):
            return

        duration_s = self.rng.uniform(0.025, 0.070)
        self.pop_events.append({
            "age": 0,
            "duration": max(8, int(duration_s * self.sample_rate)),
            "decay_s": self.rng.uniform(0.010, 0.026),
            "gain": self.crackle_gain * self.rng.uniform(0.45, 1.0)
                * (0.45 + 0.55 * self.crackle_intensity),
            "f1": self.rng.uniform(145.0, 220.0),
            "f2": self.rng.uniform(360.0, 520.0),
            "phase1": self.rng.uniform(0.0, 2.0 * math.pi),
            "phase2": self.rng.uniform(0.0, 2.0 * math.pi),
        })

    def _render_pops(self, frames, white):
        output = np.zeros(int(frames), dtype=np.float64)
        remaining = []
        offsets = np.arange(int(frames), dtype=np.int32)
        for event in self.pop_events:
            ages = event["age"] + offsets
            active = ages < event["duration"]
            if np.any(active):
                active_ages = ages[active].astype(np.float64)
                t = active_ages / self.sample_rate
                envelope = np.exp(-t / event["decay_s"])
                envelope *= 1.0 - active_ages / event["duration"]
                pulse = (
                    0.70 * white[active]
                    + 0.25 * np.sin(
                        2.0 * math.pi * event["f1"] * t + event["phase1"]
                    )
                    + 0.15 * np.sin(
                        2.0 * math.pi * event["f2"] * t + event["phase2"]
                    )
                )
                output[active] += event["gain"] * envelope * pulse
            event["age"] += int(frames)
            if event["age"] < event["duration"]:
                remaining.append(event)
        self.pop_events = remaining
        return output

    def render(
        self,
        frames,
        rpm,
        load,
        shift_kick,
        shift_blip,
        speed_kph,
        gear,
        throttle,
        acceleration_kph_s,
        signal_gain,
        top_gear,
    ):
        frames = int(frames)
        dt = frames / float(self.sample_rate)
        load = clamp(float(load), 0.0, 1.0)
        throttle = clamp(float(throttle), 0.0, 1.0)

        load_alpha = 1.0 - math.exp(-dt / 0.055)
        self.smoothed_load += load_alpha * (load - self.smoothed_load)
        load = self.smoothed_load
        if shift_kick < 0.0:
            load = max(load, 0.75 + 0.25 * clamp(shift_blip, 0.0, 1.0))

        rpm_line = np.linspace(self.previous_rpm, rpm, frames, endpoint=False)
        self.previous_rpm = float(rpm)
        shaft_hz = rpm_line / 60.0
        shaft_increment = 2.0 * math.pi * shaft_hz / self.sample_rate
        shaft_phase = self.shaft_phase + np.cumsum(shaft_increment)
        self.shaft_phase = float(shaft_phase[-1] % (2.0 * math.pi))
        white, colored = self._colored_noise(frames)
        firing_pulses = self._generate_firing_pulses(rpm_line, load, white)
        bank_left = self.bank_left_resonator.process(firing_pulses[:, 0])
        bank_right = self.bank_right_resonator.process(firing_pulses[:, 1])
        common_input = 0.5 * (bank_left + bank_right)
        common_short = self.common_resonator_short.process(common_input)
        common_long = self.common_resonator_long.process(common_input)
        common_exhaust = 0.62 * common_short + 0.38 * common_long
        exhaust_stereo = np.column_stack((
            0.70 * bank_left + 0.62 * common_exhaust,
            0.70 * bank_right + 0.62 * common_exhaust,
        ))
        exhaust_stereo = self._remove_dc(exhaust_stereo)

        # Mechanical orders remain quiet; the exhaust pulse train supplies the
        # RPM-following harmonic structure instead of a stack of pure sines.
        mechanical = (
            0.022 * np.sin(2.0 * shaft_phase + 0.30)
            + 0.016 * np.sin(4.0 * shaft_phase)
            + 0.009 * np.sin(6.0 * shaft_phase + 0.20)
        )
        intake_noise = (
            0.65 * colored + 0.10 * (white - colored)
        ) * (0.010 + 0.055 * load)
        rpm_ratio_line = np.clip(
            (rpm_line - self.idle_rpm) / (self.max_rpm - self.idle_rpm),
            0.0,
            1.0,
        )
        rpm_gain = 0.38 + 0.62 * np.sqrt(rpm_ratio_line)
        signal = rpm_gain * (
            0.5 * (exhaust_stereo[:, 0] + exhaust_stereo[:, 1])
            + mechanical
            + intake_noise
        )
        signal *= 0.62 + 0.48 * load
        bank_side = 0.16 * rpm_gain * (
            exhaust_stereo[:, 0] - exhaust_stereo[:, 1]
        )

        if shift_kick > 0.0:
            signal *= 1.0 - 0.70 * clamp(shift_kick, 0.0, 1.0)
            bank_side *= 1.0 - 0.70 * clamp(shift_kick, 0.0, 1.0)

        if abs(self.previous_shift_kick) > 0.05 and abs(shift_kick) < 0.01:
            self.gear_thump_level = 1.0
            self.thump_phase = 0.0
        self.previous_shift_kick = float(shift_kick)
        sample_index = np.arange(frames, dtype=np.float64)
        thump_envelope = self.gear_thump_level * np.exp(
            -sample_index / (self.sample_rate * 0.045)
        )
        thump_phase = self.thump_phase + (
            2.0 * math.pi * 68.0 / self.sample_rate
        ) * (sample_index + 1.0)
        signal += thump_envelope * (
            0.18 * np.sin(thump_phase)
            + 0.055 * np.sin(2.0 * thump_phase + 0.25)
            + 0.018 * white
        )
        self.thump_phase = float(thump_phase[-1] % (2.0 * math.pi))
        self.gear_thump_level = float(thump_envelope[-1])

        rpm_ratio = clamp(
            (float(rpm) - self.idle_rpm) / (self.max_rpm - self.idle_rpm),
            0.0,
            1.0,
        )
        self._update_crackle_state(
            dt, rpm, rpm_ratio, speed_kph, gear, throttle, acceleration_kph_s
        )
        self._maybe_start_pop(dt)
        signal += self._render_pops(frames, white)

        self.limiter_active = bool(
            self.enable_limiter
            and rpm > self.limiter_rpm
            and throttle > 0.75
            and gear >= top_gear
        )
        if self.limiter_active:
            limiter_phase = self.limiter_phase + (
                2.0 * math.pi * self.limiter_hz / self.sample_rate
            ) * (sample_index + 1.0)
            ignition = 0.5 + 0.5 * np.tanh(7.0 * np.sin(limiter_phase))
            signal *= 0.45 + 0.55 * ignition
            self.limiter_phase = float(limiter_phase[-1] % (2.0 * math.pi))

        output_gain = self.volume * clamp(signal_gain, 0.0, 1.0)
        side = bank_side + 0.012 * colored
        left = np.tanh((signal + side) * output_gain * 1.15) * 0.84
        right = np.tanh((signal - side) * output_gain * 1.15) * 0.84
        stereo = np.empty(frames * 2, dtype="<i2")
        stereo[0::2] = np.asarray(np.clip(left, -1.0, 1.0) * 32767.0, dtype="<i2")
        stereo[1::2] = np.asarray(np.clip(right, -1.0, 1.0) * 32767.0, dtype="<i2")
        return stereo.tobytes()


class V8EngineSoundNode:
    def __init__(self):
        nice_level = int(clamp(rospy.get_param("~nice_level", 5), 0, 15))
        if nice_level > 0:
            try:
                os.nice(nice_level)
            except OSError as error:
                rospy.logwarn("[v8_engine] could not lower process priority: %s", error)
        self.ego_topic = rospy.get_param("~ego_topic", "/morai/ego_topic")
        self.velocity_scale_to_kph = float(
            rospy.get_param("~velocity_scale_to_kph", 1.0)
        )
        self.sample_rate = int(rospy.get_param("~sample_rate", 48000))
        self.chunk_ms = float(rospy.get_param("~chunk_ms", 20.0))
        self.volume = float(rospy.get_param("~volume", 0.35))
        self.engine_style = str(
            rospy.get_param("~engine_style", "v8_procedural")
        ).lower()
        if self.engine_style not in ("v8_procedural", "v8_sample_loop"):
            raise RuntimeError(
                "v8_engine.py supports v8_procedural or v8_sample_loop"
            )
        self.engine_cylinders = int(rospy.get_param("~engine_cylinders", 8))
        self.backend = rospy.get_param("~audio_backend", "paplay")
        self.audio_latency_ms = float(rospy.get_param("~audio_latency_ms", 120.0))
        self.audio_process_time_ms = float(
            rospy.get_param("~audio_process_time_ms", 20.0)
        )
        self.state_publish_hz = max(
            1.0, float(rospy.get_param("~state_publish_hz", 5.0))
        )
        self.message_timeout_s = float(rospy.get_param("~message_timeout_s", 0.5))
        self.frames_per_chunk = max(
            128, int(self.sample_rate * self.chunk_ms / 1000.0)
        )

        self.lock = threading.Lock()
        self.speed_kph = 0.0
        self.throttle = 0.0
        self.brake = 0.0
        self.acceleration_kph_s = 0.0
        self.callback_speed_kph = 0.0
        self.callback_time = None
        self.last_message_time = None
        self.signal_gain = 0.0
        self.last_shift_label = None

        self.powertrain = CompressedICEPowertrain()
        self.synth = ProceduralV8Engine(
            self.sample_rate,
            self.volume,
            self.powertrain.idle_rpm,
            self.powertrain.max_rpm,
            self.engine_cylinders,
        )
        self.use_hybrid_samples = bool(
            rospy.get_param("~use_hybrid_samples", True)
        )
        self.use_gear_loop_sample = bool(
            rospy.get_param("~use_gear_loop_sample", True)
        )
        self.use_m3_sample = bool(
            rospy.get_param("~use_m3_sample", False)
        ) and not self.use_gear_loop_sample
        self.procedural_mix = clamp(
            float(rospy.get_param("~procedural_mix", 0.18)), 0.0, 1.0
        )
        self.startup_sample = None
        self.gear_loop_sample = None
        self.m3_sample = None
        if self.use_hybrid_samples:
            startup_path = rospy.get_param("~engine_start_audio_path")
            self.startup_sample = StartupIdleSample(
                startup_path,
                self.sample_rate,
                float(rospy.get_param("~startup_loop_start_s", 3.0)),
                float(rospy.get_param("~startup_loop_end_s", 9.5)),
                float(rospy.get_param("~startup_sample_gain", 1.15)),
            )
            if self.use_gear_loop_sample:
                gear_loop_path = rospy.get_param("~gear_loop_audio_path")
                self.gear_loop_sample = SingleGearLoopSample(
                    gear_loop_path,
                    self.sample_rate,
                    float(rospy.get_param("~gear_loop_reference_rpm", 7000.0)),
                    float(rospy.get_param("~gear_loop_start_s", 0.10)),
                    float(rospy.get_param("~gear_loop_end_s", 1.50)),
                    float(rospy.get_param("~gear_loop_crossfade_s", 0.10)),
                    float(rospy.get_param("~gear_loop_gain", 1.55)),
                    float(rospy.get_param("~gear_loop_minimum_rate", 0.24)),
                )
                rospy.loginfo(
                    "[v8_engine] startup/idle + single gear loop loaded: %s %s",
                    startup_path,
                    gear_loop_path,
                )
            elif self.use_m3_sample:
                m3_path = rospy.get_param("~m3_audio_path")
                gear_segments = rospy.get_param(
                    "~m3_gear_segments_s",
                    [
                        [1.25, 3.20], [3.35, 6.50], [6.65, 9.45],
                        [9.65, 14.35], [79.00, 84.35], [84.55, 89.50],
                    ],
                )
                overrun_segment = rospy.get_param(
                    "~m3_overrun_segment_s", [15.0, 28.0]
                )
                gear_start_rpm = rospy.get_param(
                    "~m3_gear_start_rpm",
                    [1800.0, 3900.0, 4300.0, 4700.0, 3800.0, 5000.0],
                )
                self.m3_sample = M3TransientSample(
                    m3_path,
                    self.sample_rate,
                    gear_segments,
                    overrun_segment,
                    gear_start_rpm,
                    self.powertrain.upshift_rpm,
                    float(rospy.get_param("~m3_sample_gain", 0.82)),
                )
                rospy.loginfo(
                    "[v8_engine] startup and M3 samples loaded: %s %s",
                    startup_path,
                    m3_path,
                )
            else:
                rospy.loginfo(
                    "[v8_engine] startup/idle sample loaded; M3 layer disabled: %s",
                    startup_path,
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
        self.load_pub = rospy.Publisher("~engine_load", Float64, queue_size=1)
        self.crackle_pub = rospy.Publisher("~crackle_intensity", Float64, queue_size=1)
        self.exhaust_temp_pub = rospy.Publisher("~exhaust_temp", Float64, queue_size=1)
        self.limiter_pub = rospy.Publisher("~limiter_active", Bool, queue_size=1)
        self.ego_sub = rospy.Subscriber(
            self.ego_topic, EgoVehicleStatus, self.ego_callback, queue_size=1
        )
        rospy.on_shutdown(self.audio.close)
        rospy.loginfo(
            "[v8_engine] listening=%s scale_to_kph=%.3f volume=%.2f backend=%s",
            self.ego_topic,
            self.velocity_scale_to_kph,
            self.volume,
            self.backend,
        )

    def ego_callback(self, msg):
        now = time.monotonic()
        raw_speed = math.hypot(msg.velocity.x, msg.velocity.y)
        speed_kph = max(0.0, raw_speed * self.velocity_scale_to_kph)
        with self.lock:
            if self.callback_time is not None:
                dt = max(1e-3, now - self.callback_time)
                raw_acceleration = (speed_kph - self.callback_speed_kph) / dt
                alpha = 1.0 - math.exp(-dt / 0.18)
                self.acceleration_kph_s += alpha * (
                    raw_acceleration - self.acceleration_kph_s
                )
            self.speed_kph = speed_kph
            self.callback_speed_kph = speed_kph
            self.callback_time = now
            self.throttle = clamp(msg.accel, 0.0, 1.0)
            self.brake = clamp(msg.brake, 0.0, 1.0)
            self.last_message_time = now

    def run(self):
        chunk_dt = self.frames_per_chunk / float(self.sample_rate)
        next_status_time = time.monotonic()
        top_gear = len(self.powertrain.gear_max_speeds_kph)

        while not rospy.is_shutdown():
            loop_start = time.monotonic()
            with self.lock:
                speed_kph = self.speed_kph
                throttle = self.throttle
                brake = self.brake
                acceleration_kph_s = self.acceleration_kph_s
                last_message_time = self.last_message_time

            message_fresh = (
                last_message_time is not None
                and loop_start - last_message_time <= self.message_timeout_s
            )
            target_gain = 1.0 if message_fresh else 0.0
            gain_alpha = 1.0 - math.exp(-chunk_dt / 0.12)
            self.signal_gain += gain_alpha * (target_gain - self.signal_gain)

            rpm, gear, _, shift_kick = self.powertrain.update(
                speed_kph, chunk_dt, throttle, brake
            )
            shift_label = self.powertrain.shift_label()
            if shift_label != self.last_shift_label:
                rospy.loginfo(
                    "[v8_engine] speed=%.1f kph gear=%s rpm=%.0f",
                    speed_kph,
                    shift_label,
                    rpm,
                )
                self.last_shift_label = shift_label
            load = clamp(
                0.16
                + 0.78 * throttle
                + 0.10 * max(0.0, acceleration_kph_s / 20.0),
                0.12,
                1.0,
            )
            if shift_kick > 0.0:
                load *= self.powertrain.torque_scale
            elif shift_kick < 0.0:
                load = max(
                    load, 0.75 + 0.25 * self.powertrain.shift_blip
                )
            startup = None
            drive = None
            drive_mix = 1.0
            if self.use_hybrid_samples:
                startup = self.startup_sample.render(
                    self.frames_per_chunk, speed_kph, acceleration_kph_s
                )
                drive_mix = self.startup_sample.drive_mix
                if self.gear_loop_sample is not None and drive_mix > 0.0:
                    drive = self.gear_loop_sample.render(
                        self.frames_per_chunk,
                        rpm,
                        load,
                        shift_kick,
                        self.signal_gain,
                    )
                elif self.m3_sample is not None and drive_mix > 0.0:
                    drive = self.m3_sample.render(
                        self.frames_per_chunk,
                        rpm,
                        gear,
                        throttle,
                        acceleration_kph_s,
                        shift_kick,
                    )

            render_procedural = (
                not self.use_hybrid_samples
                or (
                    drive_mix > 0.02
                    and self.gear_loop_sample is None
                    and (
                        self.m3_sample is None
                        or self.m3_sample.activity_gain < 0.95
                    )
                )
            )
            if render_procedural:
                pcm = self.synth.render(
                    self.frames_per_chunk,
                    rpm,
                    load,
                    shift_kick,
                    self.powertrain.shift_blip,
                    speed_kph,
                    gear,
                    throttle,
                    acceleration_kph_s,
                    self.signal_gain,
                    top_gear,
                )
            else:
                pcm = bytes(self.frames_per_chunk * 4)

            if self.use_hybrid_samples:
                procedural = (
                    np.frombuffer(pcm, dtype="<i2")
                    .reshape(-1, 2).astype(np.float64) / 32768.0
                )
                if self.gear_loop_sample is not None:
                    if drive is None:
                        drive = np.zeros_like(startup)
                    sample_signal = startup + drive * drive_mix
                    procedural_gain = 0.0
                elif self.m3_sample is not None:
                    if drive is None:
                        drive = np.zeros_like(startup)
                    sample_signal = startup + drive * drive_mix
                    procedural_gain = (
                        1.0 - (1.0 - self.procedural_mix)
                        * self.m3_sample.activity_gain
                    )
                else:
                    sample_signal = startup
                    procedural_gain = 0.08 + 0.92 * drive_mix
                combined = sample_signal + procedural * procedural_gain
                combined = np.tanh(combined * 1.05) * 0.92
                stereo = np.asarray(
                    np.clip(combined, -1.0, 1.0) * 32767.0,
                    dtype="<i2",
                )
                pcm = stereo.reshape(-1).tobytes()
            try:
                self.audio.write(pcm)
            except (BrokenPipeError, OSError, RuntimeError) as error:
                if rospy.is_shutdown():
                    return
                rospy.logerr("[v8_engine] audio output failed: %s", error)
                self.audio.close()
                return

            now = time.monotonic()
            if now >= next_status_time:
                self.rpm_pub.publish(Float64(data=rpm))
                self.gear_pub.publish(Int16(data=gear))
                self.shift_state_pub.publish(String(data=shift_label))
                self.blip_pub.publish(Float64(data=self.powertrain.shift_blip))
                self.torque_scale_pub.publish(
                    Float64(data=self.powertrain.torque_scale)
                )
                self.speed_pub.publish(Float64(data=speed_kph))
                self.load_pub.publish(Float64(data=load))
                self.crackle_pub.publish(
                    Float64(data=self.synth.crackle_intensity)
                )
                self.exhaust_temp_pub.publish(
                    Float64(data=self.synth.exhaust_temp)
                )
                self.limiter_pub.publish(Bool(data=self.synth.limiter_active))
                rospy.loginfo(
                    "[v8_engine] speed=%.1f kph gear=%s rpm=%.0f load=%.2f",
                    speed_kph,
                    shift_label,
                    rpm,
                    load,
                )
                next_status_time = now + 1.0 / self.state_publish_hz

            elapsed = time.monotonic() - loop_start
            if elapsed < chunk_dt:
                time.sleep(chunk_dt - elapsed)


def main():
    rospy.init_node("morai_engine_sound", anonymous=False)
    try:
        node = V8EngineSoundNode()
        node.run()
    except (RuntimeError, OSError) as error:
        rospy.logfatal("[v8_engine] startup failed: %s", error)


if __name__ == "__main__":
    main()
