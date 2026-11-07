#!/bin/sh
# Pauses / resumes the Kindle UI around kdc sessions that span several
# commands (e.g. leaving calibration ladders on screen).
#
#   kdc-session.sh pause     SIGSTOP the UI (cvm, awesome), keep screensaver off
#   kdc-session.sh resume    SIGCONT what pause stopped, screensaver back on
set -eu
STATE=${KDC_STATE:-/var/local/kdc}
PIDS=$STATE/paused.pids
mkdir -p "$STATE"

case "${1:-}" in
pause)
	[ -s "$PIDS" ] && { echo "kdc-session: already paused ($(cat "$PIDS"))"; exit 0; }
	lipc-set-prop com.lab126.powerd preventScreenSaver 1 >/dev/null 2>&1 || true
	: > "$PIDS"
	for name in cvm awesome; do
		for p in $(pidof "$name" || true); do
			kill -STOP "$p" && echo "$p" >> "$PIDS"
		done
	done
	echo "kdc-session: paused $(tr '\n' ' ' < "$PIDS")"
	;;
resume)
	if [ -f "$PIDS" ]; then
		for p in $(cat "$PIDS"); do kill -CONT "$p" 2>/dev/null || true; done
		rm -f "$PIDS"
	fi
	lipc-set-prop com.lab126.powerd preventScreenSaver 0 >/dev/null 2>&1 || true
	echo "kdc-session: resumed"
	;;
*)
	echo "usage: $0 pause | resume" >&2
	exit 2
	;;
esac
