from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.responses import FileResponse, RedirectResponse
from pydantic import BaseModel, Field
import qn8027
from signal_generator import main as signal_generator_service


class FmSettings(BaseModel):
    pa_target: int | None = Field(default=None, ge=20, le=75)
    rin: int | None = Field(default=None, ge=0, le=3)
    gvga: int | None = Field(default=None, ge=0, le=5)
    gdb: int | None = Field(default=None, ge=0, le=2)
    xtal: int | None = Field(default=None, ge=0, le=1)
    clock_source: int | None = Field(default=None, ge=0, le=3)
    oscillator_current_code: int | None = Field(default=None, ge=0, le=63)
    preemphasis: int | None = Field(default=None, ge=0, le=1)
    pa_timeout: int | None = Field(default=None, ge=0, le=3)
    pilot_gain: int | None = Field(default=None, ge=7, le=10)
    privacy_mode: bool | None = None
    mute: bool | None = None
    tx_fdev_code: int | None = Field(default=None, ge=0, le=255)
    rds_enabled: bool | None = None
    rds_deviation_code: int | None = Field(default=None, ge=0, le=127)

@asynccontextmanager
async def lifespan(_: FastAPI):
    try:
        yield
    finally:
        signal_generator_service.generator.stop()

app = FastAPI(lifespan=lifespan)
qn = qn8027.qn8027()
app.mount("/signal-generator", signal_generator_service.app, name="signal-generator")

@app.get("/signal-generator", include_in_schema=False)
async def signal_generator_redirect():
    return RedirectResponse("/signal-generator/")

@app.get("/", include_in_schema=False)
async def dashboard():
    return FileResponse(signal_generator_service.STATIC_DIR / "index.html")

@app.post("/fmtransmitter/reset_chip")
async def reset_chip():
    qn.reset_chip()

@app.post("/fmtransmitter/recalibrate")
async def recalibrate():
    qn.start_fsm_recalibration()

@app.post("/fmtransmitter/initialize")
async def initialize():
    qn.default_init()

@app.post("/fmtransmitter/enable_tx")
async def enable_tx():
    qn.set_enable_tx(True)

@app.post("/fmtransmitter/disable_tx")
async def disable_tx():
    qn.set_enable_tx(False)

@app.post("/fmtransmitter/set_freq/{freqMHz}")
async def set_freq(freqMHz: float):
    qn.set_freq(freqMHz)

@app.post("/fmtransmitter/stereo")
async def stereo():
    qn.set_enable_stereo(True)

@app.post("/fmtransmitter/mono")
async def mono():
    qn.set_enable_stereo(False)

@app.post("/fmtransmitter/mute")
async def stereo():
    qn.set_enable_mute(True)

@app.post("/fmtransmitter/unmute")
async def mono():
    qn.set_enable_mute(False)

@app.post("/fmtransmitter/settings")
async def set_settings(settings: FmSettings):
    if settings.pa_target is not None:
        qn.set_pac(settings.pa_target)
        state = qn.get_state()
        if qn8027.TXREQ(state.txreq.value) == qn8027.TXREQ.TRANSMIT:
            qn.set_enable_tx(False)
            qn.set_enable_tx(True)
    if settings.rin is not None:
        qn.set_rin(qn8027.VGA_R_IN(settings.rin))
    if settings.gvga is not None:
        qn.set_gvga(qn8027.VGA_GVGA_LEVEL(settings.gvga))
    if settings.gdb is not None:
        qn.set_gdb(qn8027.VGA_GDB(settings.gdb))
    if any(value is not None for value in (settings.xtal, settings.clock_source, settings.oscillator_current_code)):
        current = qn.get_state()
        qn.set_xtal(
            xtal=qn8027.VGA_XSEL(settings.xtal if settings.xtal is not None else current.xsel.value),
            xinj=qn8027.XTL_XINJ(settings.clock_source if settings.clock_source is not None else current.xinj.value),
            xisel=settings.oscillator_current_code if settings.oscillator_current_code is not None else round(current.xisel / 6.25),
        )
    if any(value is not None for value in (settings.preemphasis, settings.pa_timeout, settings.pilot_gain, settings.privacy_mode)):
        current = qn.get_state()
        qn.set_gplt(
            tc=qn8027.TC(settings.preemphasis if settings.preemphasis is not None else current.tc.value),
            t1m1=qn8027.GPLT_T1MSEL(settings.pa_timeout if settings.pa_timeout is not None else current.t1m_sel.value),
            gain_txplt=qn8027.GPLT_GAINTXPLT(settings.pilot_gain if settings.pilot_gain is not None else current.gain_txplt.value),
            priv_en=qn8027.PRIVEN(int(settings.privacy_mode) if settings.privacy_mode is not None else current.priv_en.value),
        )
    if settings.mute is not None:
        qn.set_enable_mute(settings.mute)
    if settings.tx_fdev_code is not None:
        qn.set_tx_fdev(settings.tx_fdev_code)
    if settings.rds_enabled is not None or settings.rds_deviation_code is not None:
        current = qn.get_state()
        qn.set_rds(
            enabled=settings.rds_enabled if settings.rds_enabled is not None else current.rdsen.value,
            deviation=settings.rds_deviation_code if settings.rds_deviation_code is not None else round(current.rdsfdev / 0.35),
        )
    return qn.get_state()

@app.post("/fmtransmitter/pa_target/{pa_target_value}")
async def set_pa_target(pa_target_value: int):
    qn.set_pac(pa_target_value)
    s = qn.get_state()
    if qn8027.TXREQ(s.txreq.value) == qn8027.TXREQ.TRANSMIT:
        qn.set_enable_tx(False)
        qn.set_enable_tx(True)
    return { "pa_target_dbu": s.pa_trgt }

@app.post("/fmtransmitter/clear_apk")
async def clear_apk():
    qn.reset_aud_apk()

@app.get("/fmtransmitter/get_apk")
async def get_apk():
    return { "aud_peak_level": qn.get_state().aud_pk}

@app.get("/fmtransmitter/get_state")
async def get_state():
    return qn.get_state()