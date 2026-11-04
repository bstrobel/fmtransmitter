"""
Signal Generator Service
========================
Setup on Raspberry Pi / Ubuntu with PulseAudio:
  1. pip install -r requirements.txt
  2. python main.py --list-sinks         # list available PulseAudio sinks
  3. python main.py --sink alsa_output.pci-0000_00_1f.3.analog-stereo

Requires: pacat (pulseaudio-utils) — usually pre-installed with PulseAudio.

Swagger UI: http://<host>:<port>/docs
ReDoc:       http://<host>:<port>/redoc

Run as a service: see signal-generator.service
"""

import argparse
import os
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from .generator import SignalGenerator

STATIC_DIR = Path(__file__).parent / "static"

app = FastAPI(
    title="Signal Generator",
    version="1.0.0",
    description="""
Audio signal generator running on PulseAudio.

Generates **sine**, **triangle**, and **rectangle** waveforms with
adjustable frequency (20 Hz – 20 kHz) and amplitude.
Supports one-shot and looping **frequency sweeps** (linear or logarithmic).

### Quick start
```
POST /api/params   { "waveform": "sine", "frequency": 1000, "amplitude": 0.5 }
POST /api/start
```

### Frequency sweep
```
POST /api/sweep    { "start_freq": 100, "end_freq": 10000, "duration": 5 }
DELETE /api/sweep  (stop sweep, keep audio running)
POST /api/stop     (stop audio entirely)
```
""",
    contact={"name": "Signal Generator"},
    license_info={"name": "MIT"},
)

generator = SignalGenerator(
    sample_rate=int(os.getenv("SIGNAL_GENERATOR_SAMPLE_RATE", "48000")),
    sink=os.getenv("SIGNAL_GENERATOR_SINK", ""),
)


# ── Response model ─────────────────────────────────────────────────────────────

class SweepStatus(BaseModel):
    active: bool = Field(..., description="Whether a sweep is currently in progress")
    start_freq: float = Field(..., description="Sweep start frequency in Hz")
    end_freq: float = Field(..., description="Sweep end frequency in Hz")
    duration: float = Field(..., description="Total sweep duration in seconds")
    loop: bool = Field(..., description="Whether the sweep repeats automatically")
    log_scale: bool = Field(..., description="True = logarithmic frequency curve, False = linear")
    progress: float = Field(..., ge=0.0, le=1.0, description="Sweep progress from 0.0 to 1.0")


class GeneratorStatus(BaseModel):
    running: bool = Field(..., description="True if audio output is active")
    waveform: str = Field(..., description="Active waveform: sine | triangle | rectangle")
    frequency: float = Field(..., description="Current output frequency in Hz (tracks sweep when active)")
    amplitude: float = Field(..., ge=0.0, le=1.0, description="Output amplitude 0.0 – 1.0")
    sample_rate: int = Field(..., description="DAC sample rate in Hz")
    sink: str = Field(..., description="PulseAudio sink in use ('default' = system default)")
    sweep: Optional[SweepStatus] = Field(None, description="Sweep state, null when no sweep is active")


class Sink(BaseModel):
    index: str = Field(..., description="PulseAudio sink index")
    name: str = Field(..., description="PulseAudio sink name (pass to --sink / /api/params)")


# ── Request models ─────────────────────────────────────────────────────────────

class Params(BaseModel):
    model_config = {"json_schema_extra": {"example": {"waveform": "sine", "frequency": 1000, "amplitude": 0.5}}}

    waveform: Optional[str] = Field(
        None,
        description="Waveform shape: `sine`, `triangle`, or `rectangle`",
        pattern="^(sine|triangle|rectangle)$",
    )
    frequency: Optional[float] = Field(
        None, ge=20, le=20000,
        description="Output frequency in Hz (20 – 20 000)",
    )
    amplitude: Optional[float] = Field(
        None, ge=0.0, le=1.0,
        description="Output amplitude (0.0 = silence, 1.0 = full scale)",
    )


class SweepParams(BaseModel):
    model_config = {"json_schema_extra": {"example": {
        "start_freq": 100, "end_freq": 10000, "duration": 5,
        "waveform": "sine", "loop": False, "log_scale": True,
    }}}

    start_freq: float = Field(..., ge=20, le=20000, description="Sweep start frequency in Hz")
    end_freq: float = Field(..., ge=20, le=20000, description="Sweep end frequency in Hz")
    duration: float = Field(..., ge=0.05, le=300, description="Sweep duration in seconds (0.05 – 300)")
    waveform: Optional[str] = Field(
        None,
        description="Waveform to use during sweep; inherits current waveform if omitted",
        pattern="^(sine|triangle|rectangle)$",
    )
    loop: bool = Field(False, description="Repeat the sweep indefinitely until stopped")
    log_scale: bool = Field(
        False,
        description=(
            "Frequency curve: `false` = linear (equal Hz/s), "
            "`true` = logarithmic (equal octaves/s — more natural for audio)"
        ),
    )


# ── Routes ─────────────────────────────────────────────────────────────────────

_STATUS_RESPONSES: Dict[int, Any] = {200: {"model": GeneratorStatus}}


@app.get(
    "/api/status",
    response_model=GeneratorStatus,
    summary="Get current generator state",
    description="Returns the complete state of the generator including waveform, frequency, amplitude, and sweep progress.",
    tags=["Generator"],
)
def get_status():
    return generator.status()


@app.post(
    "/api/start",
    response_model=GeneratorStatus,
    summary="Start audio output",
    description="Opens a PulseAudio stream and begins outputting the configured waveform. No-op if already running.",
    tags=["Generator"],
    responses={500: {"description": "PulseAudio stream could not be opened"}},
)
def start():
    try:
        generator.start()
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc))
    return generator.status()


@app.post(
    "/api/stop",
    response_model=GeneratorStatus,
    summary="Stop audio output",
    description="Stops and closes the PulseAudio stream. Also cancels any active sweep. No-op if already stopped.",
    tags=["Generator"],
)
def stop():
    generator.stop()
    return generator.status()


@app.put(
    "/api/params",
    response_model=GeneratorStatus,
    summary="Update generator parameters",
    description=(
        "Updates waveform, frequency, and/or amplitude. "
        "All fields are optional — omit any field to leave it unchanged. "
        "Changes take effect immediately, even while audio is playing."
    ),
    tags=["Generator"],
    responses={422: {"description": "One or more parameter values are out of range"}},
)
def set_params(params: Params):
    try:
        generator.set_params(
            waveform=params.waveform,
            frequency=params.frequency,
            amplitude=params.amplitude,
        )
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc))
    return generator.status()


@app.post(
    "/api/sweep",
    response_model=GeneratorStatus,
    summary="Start a frequency sweep",
    description=(
        "Begins a frequency sweep from `start_freq` to `end_freq` over `duration` seconds. "
        "Starts audio output automatically if the generator is not already running. "
        "The frequency curve is linear by default; set `log_scale=true` for a logarithmic "
        "(equal-octave) sweep which sounds more uniform to the human ear. "
        "When `loop=true` the sweep repeats until `DELETE /api/sweep` or `POST /api/stop` is called."
    ),
    tags=["Sweep"],
    responses={422: {"description": "One or more sweep parameters are out of range"}},
)
def start_sweep(params: SweepParams):
    try:
        generator.start_sweep(
            start_freq=params.start_freq,
            end_freq=params.end_freq,
            duration=params.duration,
            loop=params.loop,
            log_scale=params.log_scale,
            waveform=params.waveform,
        )
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc))
    return generator.status()


@app.delete(
    "/api/sweep",
    response_model=GeneratorStatus,
    summary="Stop sweep (keep audio running)",
    description=(
        "Cancels the active sweep and returns to a fixed frequency at the point "
        "where the sweep was interrupted. Audio output continues. "
        "No-op if no sweep is active."
    ),
    tags=["Sweep"],
)
def stop_sweep():
    generator.stop_sweep()
    return generator.status()


@app.get(
    "/api/sinks",
    response_model=List[Sink],
    summary="List PulseAudio output sinks",
    description=(
        "Returns all PulseAudio output sinks reported by `pactl list sinks short`. "
        "Pass the `name` of the desired sink to the `--sink` CLI argument or configure "
        "it in the systemd service file."
    ),
    tags=["System"],
)
def list_sinks():
    import subprocess
    result = subprocess.run(
        ['pactl', 'list', 'sinks', 'short'],
        capture_output=True, text=True,
    )
    sinks = []
    for line in result.stdout.strip().splitlines():
        parts = line.split('\t')
        if len(parts) >= 2:
            sinks.append({"index": parts[0], "name": parts[1]})
    return sinks


@app.get("/", include_in_schema=False)
def index():
    return FileResponse(STATIC_DIR / "index.html")


app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    global generator

    parser = argparse.ArgumentParser(description="Signal Generator REST Service")
    parser.add_argument("--host", default="0.0.0.0", help="Bind host (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8000, help="Bind port (default: 8000)")
    parser.add_argument("--sink", default="", help="PulseAudio sink name (default: PulseAudio default sink)")
    parser.add_argument("--sample-rate", type=int, default=48000, help="Sample rate in Hz (default: 48000)")
    parser.add_argument("--list-sinks", action="store_true", help="List available PulseAudio sinks and exit")
    args = parser.parse_args()

    if args.list_sinks:
        import subprocess
        result = subprocess.run(['pactl', 'list', 'sinks', 'short'], capture_output=True, text=True)
        print("\nAvailable PulseAudio sinks:")
        for line in result.stdout.strip().splitlines():
            parts = line.split('\t')
            if len(parts) >= 2:
                print(f"  [{parts[0]:>3}]  {parts[1]}")
        sys.exit(0)

    generator = SignalGenerator(
        sample_rate=args.sample_rate,
        sink=args.sink,
    )

    print(f"Signal Generator listening on http://{args.host}:{args.port}")
    print(f"Swagger UI: http://{args.host}:{args.port}/docs")
    print(f"PulseAudio sink: {args.sink!r}, sample rate: {args.sample_rate} Hz")
    uvicorn.run(app, host=args.host, port=args.port, log_level="info")


if __name__ == "__main__":
    main()
