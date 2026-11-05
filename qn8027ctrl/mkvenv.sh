#!/bin/bash
VENVNAME=fmtrans
MYDIR=$(dirname $0)
cd ${MYDIR}
if [ ! -d "${VENVNAME}" ]; then
    rm -rf ${VENVNAME}
fi
python3 -m venv ${VENVNAME}
source ${VENVNAME}/bin/activate
pip install --upgrade pip
pip install -r requirements.txt