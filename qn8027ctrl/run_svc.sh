#!/bin/bash
source fmtrans/bin/activate
uvicorn qn8027rest:app --host 0.0.0.0
