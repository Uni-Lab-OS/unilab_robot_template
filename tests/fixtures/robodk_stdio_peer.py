"""Offline protocol peer, NOT a controller emulator or vendor-driver substitute.

Expected requests are written by the independent conformance test. No sockets,
RoboDK import, vendor SDK, robot or hardware is used.
"""
import json
from pathlib import Path
import sys
import time

scenario=json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
print('SMS:Disconnected',flush=True)
print('SMS:Ready',flush=True)  # Stale startup Ready must never finish CONNECT.
for expected in scenario:
 line=sys.stdin.readline().strip()
 if line != expected['request']:
  print('SMS:Error unexpected request '+repr(line),flush=True)
  sys.exit(2)
 if expected.get('delay'):time.sleep(expected['delay'])
 for response in expected.get('responses',[]):print(response,flush=True)
 if expected.get('exit'):sys.exit(3)
# Keep the pipe open until adapter terminates it; don't invent final Ready.
sys.stdin.read()
