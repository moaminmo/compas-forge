#! python 3
"""Read-only runtime probe; launch in a dedicated Rhino test process."""
import json
import os
import sys
import Rhino

result = dict(python=sys.version, executable=sys.executable,
              rhino=str(Rhino.RhinoApp.Version))
print(result)
with open(os.environ['COMPAS_FORGE_PROBE_RESULT'], 'w', encoding='utf8') as stream:
    json.dump(result,stream,indent=2)
