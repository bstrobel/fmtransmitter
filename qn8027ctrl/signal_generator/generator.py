import subprocess
import threading
import numpy as np


class SignalGenerator:
    WAVEFORMS = ('sine', 'triangle', 'rectangle')

    def __init__(self, sample_rate: int = 48000, sink: str = ''):
        self.sample_rate = sample_rate
        self.sink = sink
        self._frequency = 1000.0
        self._amplitude = 0.5
        self._waveform = 'sine'
        self._running = False
        self._proc = None
        self._thread = None
        self._phase = 0.0
        self._lock = threading.Lock()
        # Sweep state
        self._sweep_active = False
        self._sweep_start_freq = 20.0
        self._sweep_end_freq = 20000.0
        self._sweep_duration = 5.0
        self._sweep_loop = False
        self._sweep_log = False
        self._sweep_sample = 0  # current sample index within the sweep

    # ── Parameter control ─────────────────────────────────────────────────────

    def set_params(self, waveform=None, frequency=None, amplitude=None):
        if waveform is not None and waveform not in self.WAVEFORMS:
            raise ValueError(f"waveform must be one of {self.WAVEFORMS}")
        if frequency is not None and not (20.0 <= float(frequency) <= 20000.0):
            raise ValueError("frequency must be between 20 and 20000 Hz")
        if amplitude is not None and not (0.0 <= float(amplitude) <= 1.0):
            raise ValueError("amplitude must be between 0.0 and 1.0")
        with self._lock:
            if waveform is not None:
                self._waveform = waveform
            if frequency is not None:
                self._frequency = float(frequency)
            if amplitude is not None:
                self._amplitude = float(amplitude)

    def start_sweep(self, start_freq, end_freq, duration,
                    loop=False, log_scale=False, waveform=None):
        if not (20.0 <= float(start_freq) <= 20000.0):
            raise ValueError("start_freq must be between 20 and 20000 Hz")
        if not (20.0 <= float(end_freq) <= 20000.0):
            raise ValueError("end_freq must be between 20 and 20000 Hz")
        if not (0.05 <= float(duration) <= 300.0):
            raise ValueError("duration must be between 0.05 and 300 s")
        if waveform is not None and waveform not in self.WAVEFORMS:
            raise ValueError(f"waveform must be one of {self.WAVEFORMS}")
        with self._lock:
            if waveform is not None:
                self._waveform = waveform
            self._sweep_start_freq = float(start_freq)
            self._sweep_end_freq = float(end_freq)
            self._sweep_duration = float(duration)
            self._sweep_loop = bool(loop)
            self._sweep_log = bool(log_scale)
            self._sweep_sample = 0
            self._sweep_active = True
        if not self._running:
            self.start()

    def stop_sweep(self):
        """Cancel sweep; generator keeps running at the last frequency."""
        self._sweep_active = False

    # ── Audio chunk generation ─────────────────────────────────────────────────

    def _make_chunk(self, frames: int) -> bytes:
        # Scalar reads of Python floats/bools are effectively atomic in CPython
        amp = self._amplitude
        waveform = self._waveform

        if self._sweep_active:
            f1 = self._sweep_start_freq
            f2 = self._sweep_end_freq
            N = max(int(self._sweep_duration * self.sample_rate), 1)

            n = self._sweep_sample + np.arange(frames, dtype=np.float64)
            n_eff = np.minimum(n, float(N - 1))
            t = n_eff / max(N - 1, 1)  # normalized position [0, 1]

            if self._sweep_log and f1 > 0 and f2 > 0:
                inst_freq = f1 * np.power(f2 / f1, t)
            else:
                inst_freq = f1 + (f2 - f1) * t

            # Update displayed frequency to midpoint of this chunk
            self._frequency = float(inst_freq[frames // 2])

            # Phase accumulation for chirp: Σ f(n)/sr for each sample
            delta_phi = inst_freq / self.sample_rate
            phases = (self._phase + np.concatenate([[0.0], np.cumsum(delta_phi[:-1])])) % 1.0
            self._phase = float((self._phase + delta_phi.sum()) % 1.0)

            self._sweep_sample += frames
            if self._sweep_sample >= N:
                if self._sweep_loop:
                    self._sweep_sample = 0
                else:
                    self._frequency = f2
                    self._sweep_active = False
        else:
            freq = self._frequency
            phases = (self._phase + np.arange(frames, dtype=np.float64) * freq / self.sample_rate) % 1.0
            self._phase = float((phases[-1] + freq / self.sample_rate) % 1.0)

        if waveform == 'sine':
            signal = amp * np.sin(2.0 * np.pi * phases)
        elif waveform == 'triangle':
            signal = amp * (2.0 * np.abs(2.0 * phases - 1.0) - 1.0)
        else:  # rectangle
            signal = np.where(phases < 0.5, amp, -amp)

        stereo = np.column_stack([signal, signal]).astype('<f4')
        return stereo.tobytes()

    # ── Stream lifecycle ───────────────────────────────────────────────────────

    def _write_loop(self):
        chunk_frames = self.sample_rate // 20  # 50 ms chunks
        while self._running:
            chunk = self._make_chunk(chunk_frames)
            try:
                self._proc.stdin.write(chunk)
                self._proc.stdin.flush()
            except (BrokenPipeError, OSError):
                break
        try:
            self._proc.stdin.close()
        except Exception:
            pass

    def start(self):
        if self._running:
            return
        self._phase = 0.0
        cmd = [
            'pacat', '--playback',
            f'--rate={self.sample_rate}',
            '--channels=2',
            '--format=float32le',
            '--latency-msec=100',
        ]
        if self.sink:
            cmd += ['--device', self.sink]
        self._proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        self._running = True
        self._thread = threading.Thread(target=self._write_loop, daemon=True)
        self._thread.start()

    def stop(self):
        if not self._running:
            return
        self._sweep_active = False
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
            self._thread = None
        if self._proc:
            self._proc.terminate()
            self._proc.wait(timeout=2)
            self._proc = None

    # ── Status ─────────────────────────────────────────────────────────────────

    def status(self) -> dict:
        sweep = None
        if self._sweep_active:
            N = max(int(self._sweep_duration * self.sample_rate), 1)
            sweep = {
                'active': True,
                'start_freq': self._sweep_start_freq,
                'end_freq': self._sweep_end_freq,
                'duration': self._sweep_duration,
                'loop': self._sweep_loop,
                'log_scale': self._sweep_log,
                'progress': min(1.0, self._sweep_sample / N),
            }
        return {
            'running': self._running,
            'waveform': self._waveform,
            'frequency': self._frequency,
            'amplitude': self._amplitude,
            'sample_rate': self.sample_rate,
            'sink': self.sink or 'default',
            'sweep': sweep,
        }
