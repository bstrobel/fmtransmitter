from fastapi import FastAPI
from qn8027 import qn8027

app = FastAPI()
qn = qn8027()

@app.get("/fmtransmitter/initialize")
async def enable_tx():
    qn.default_init()

@app.get("/fmtransmitter/reset_chip")
async def enable_tx():
    qn.reset_chip()

@app.get("/fmtransmitter/enable_tx")
async def enable_tx():
    qn.set_enable_tx(True)


@app.get("/fmtransmitter/disable_tx")
async def disable_tx():
    qn.set_enable_tx(False)
