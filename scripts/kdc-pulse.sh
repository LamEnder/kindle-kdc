#!/bin/sh
# Installs or removes the kdc pulse waveform on a Kindle PaperWhite 4.
#
#   kdc-pulse.sh install [FRAMES [LONG]]  patch the loaded waveform, reload it
#                                   (LONG: also put long pulses of LONG frames in
#                                   the DU slot = the dual-pulse waveform)
#   kdc-pulse.sh restore            reload the saved stock waveform
#   kdc-pulse.sh status
#
# How it works: the Lab126 EPDC driver reloads its waveform from
# /mnt/wfm/waveform_to_use/*.wrf when "reload" is written to
# /proc/eink/waveform/source. This script saves the loaded waveform from
# /proc/eink/waveform/data, replaces the DU4 slot with the pulse waveform,
# serves the result from a tmpfs mounted over that directory, and reloads.
# The stock file on the device is never modified; a reboot always restores
# the stock waveform too.
#
# Run as root with the screen idle (KOReader/the Kindle UI not refreshing).
set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
KDCWFM=${KDCWFM:-$HERE/kdc-wfm}
STATE=${KDC_STATE:-/var/local/kdc}
PROC_DATA=/proc/eink/waveform/data
PROC_SRC=/proc/eink/waveform/source
WFM_PARENT=/mnt/wfm
WFM_DIR=$WFM_PARENT/waveform_to_use
TAG=kdcwfm

die() { echo "kdc-pulse: $*" >&2; exit 1; }
say() { echo "kdc-pulse: $*"; }

[ "$(id -u)" = 0 ] || die "must run as root"
[ -x "$KDCWFM" ] || die "kdc-wfm not found at $KDCWFM (set KDCWFM=...)"
[ -r "$PROC_DATA" ] && [ -e "$PROC_SRC" ] || die "$PROC_DATA / $PROC_SRC missing: not a Lab126 EPDC kernel?"
mkdir -p "$STATE"

slots() {
	eval "$("$KDCWFM" probe)"   # sets du4=N du=N
	[ -n "${du4:-}" ] && [ -n "${du:-}" ] || die "probe failed"
}

ours_mounted() { grep -q "^$TAG $1 " /proc/mounts; }

# Make $WFM_DIR a tmpfs we own. Records what was mounted in $STATE/mounts.
prepare_dir() {
	ours_mounted "$WFM_DIR" && return 0
	: > "$STATE/mounts.new"
	if [ ! -d "$WFM_DIR" ]; then
		[ -d "$WFM_PARENT" ] || die "$WFM_PARENT does not exist; run 'mntroot rw', mkdir $WFM_PARENT, 'mntroot ro', then retry"
		if ! mkdir "$WFM_DIR" 2>/dev/null; then
			# Read-only parent: put a tmpfs over the parent first.
			mount -t tmpfs -o size=1m,mode=755 "$TAG" "$WFM_PARENT"
			echo "$WFM_PARENT" >> "$STATE/mounts.new"
			mkdir "$WFM_DIR"
		fi
	fi
	mount -t tmpfs -o size=8m,mode=755 "$TAG" "$WFM_DIR"
	echo "$WFM_DIR" >> "$STATE/mounts.new"
	cat "$STATE/mounts.new" >> "$STATE/mounts"
	rm -f "$STATE/mounts.new"
}

release_dir() {
	[ -f "$STATE/mounts" ] || return 0
	# Unmount in reverse order.
	for m in $(sed '1!G;h;$!d' "$STATE/mounts"); do
		ours_mounted "$m" && umount "$m"
	done
	rm -f "$STATE/mounts"
}

load_file() {   # $1 = waveform file to serve and reload
	# Never change the waveform size: the Lab126 reload path frees its DMA
	# buffer with the last recorded size, and a grow after a shrink hard-froze
	# a PW4. kdc-wfm patches in place, so pulse and stock are the same size.
	want=$(wc -c < "$1")
	have=$(cat "$PROC_DATA" | wc -c)
	[ "$want" -eq "$have" ] || die "refusing to reload: $1 is $want bytes, loaded waveform is $have (sizes must match)"
	prepare_dir
	rm -f "$WFM_DIR"/*
	cp "$1" "$WFM_DIR/kdc.wrf"
	sync
	echo reload > "$PROC_SRC" || die "writing $PROC_SRC failed"
	sleep 1
	cat "$PROC_DATA" > "$STATE/loaded.wrf"
	cmp -s "$STATE/loaded.wrf" "$1"
}

cmd_install() {
	frames=${1:-3}
	long=${2:-0}
	slots
	say "DU4 requests use waveform slot $du4, DU uses slot $du"
	cat "$PROC_DATA" > "$STATE/current.wrf"
	if "$KDCWFM" verify "$STATE/current.wrf" --slot "$du4" >/dev/null; then
		[ -f "$STATE/stock.wrf" ] || die "pulse waveform is loaded but $STATE/stock.wrf is missing; reboot to restore stock"
		say "pulse waveform already loaded; re-patching from the saved stock waveform"
	else
		cp "$STATE/current.wrf" "$STATE/stock.wrf"
		say "saved stock waveform to $STATE/stock.wrf"
	fi
	if [ "$long" -gt 0 ]; then
		"$KDCWFM" patch "$STATE/stock.wrf" "$STATE/pulse.wrf" --slot "$du4" --ref-slot "$du" --frames "$frames" \
			--long-slot "$du" --long-frames "$long"
	else
		"$KDCWFM" patch "$STATE/stock.wrf" "$STATE/pulse.wrf" --slot "$du4" --ref-slot "$du" --frames "$frames"
	fi
	if load_file "$STATE/pulse.wrf"; then
		say "pulse waveform active (slot $du4). Run 'kdc-pulse.sh restore' when done."
	else
		say "loaded waveform does not match; restoring stock"
		load_file "$STATE/stock.wrf" || die "restore failed too: reboot the Kindle"
		release_dir
		exit 1
	fi
}

cmd_restore() {
	[ -f "$STATE/stock.wrf" ] || die "no saved stock waveform; reboot to restore"
	if load_file "$STATE/stock.wrf"; then
		release_dir
		say "stock waveform restored"
	else
		die "stock waveform did not reload cleanly: reboot the Kindle"
	fi
}

cmd_status() {
	slots
	cat "$PROC_DATA" > "$STATE/current.wrf"
	echo "slots: du4=$du4 du=$du"
	echo "source: $(cat "$PROC_SRC" 2>/dev/null || echo '?')"
	echo "loaded: $("$KDCWFM" verify "$STATE/current.wrf" --slot "$du4" || true)"
	echo "mounts: $(grep "^$TAG " /proc/mounts | awk '{print $2}' | tr '\n' ' ')"
}

case "${1:-}" in
	install) shift; cmd_install "$@" ;;
	restore) cmd_restore ;;
	status) cmd_status ;;
	*) echo "usage: $0 install [FRAMES [LONG]] | restore | status" >&2; exit 2 ;;
esac
