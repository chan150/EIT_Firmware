#!/bin/bash
# jlc.cfg is local configuration, not tracked. Say so plainly rather than
# letting J-Link fail on a missing script.
[ -f jlc.cfg ] || { echo "jlc.cfg missing - cp jlc.cfg.example jlc.cfg" >&2; exit 1; }
JLinkExe -device ADUCM350 -if SWD -speed 115.2 -jtagconf -1,-1 -autoconnect 1 -CommanderScript jlc.cfg
