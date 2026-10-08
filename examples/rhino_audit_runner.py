#! python 3
"""Capture in-host failures as evidence, not a missing/ambiguous result file."""
import json
import os
import runpy
import traceback

try:
    runpy.run_path(os.environ['COMPAS_FORGE_SMOKE_SCRIPT'], run_name='__main__')
except BaseException:
    with open(os.environ['COMPAS_FORGE_SMOKE_RESULT'], 'w', encoding='utf8') as stream:
        json.dump(dict(passed=False, traceback=traceback.format_exc()), stream, indent=2)
    raise
