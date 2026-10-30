#!/bin/bash
# set -x
MYDIR=$(dirname $0)
cd ${MYDIR}
source fmtrans/bin/activate
exec uvicorn qn8027rest:app --host 0.0.0.0
