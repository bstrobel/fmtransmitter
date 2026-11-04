from contextlib import asynccontextmanager
from pathlib import Path
from fastapi import FastAPI
from fastapi.responses import FileResponse, RedirectResponse
import qn8027
from signal_generator import main as signal_generator_service

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
    return FileResponse(Path(__file__).with_name("index.html"))

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