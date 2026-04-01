#!/usr/bin/env bash
set -euo pipefail

SESH="unitree_g1"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NET_IF=$(ip route get 8.8.8.8 | awk '{print $5; exit}')
cd "$SCRIPT_DIR"

log()  { echo -e "\n\033[1;32m[+] $*\033[0m"; }
die()  { echo -e "\n\033[1;31m[✗] $*\033[0m" >&2; exit 1; }

need_cmd() { command -v "$1" >/dev/null 2>&1 || die "Missing command: $1"; }

tmux_send() {
  local target="$1"; shift
  tmux send-keys -t "$target" "$*" C-m
}

find_keyboard_event_device() {
  local dev

  # 1) Prefer stable by-id symlinks
  for dev in /dev/input/by-id/*-event-kbd; do
    [[ -e "$dev" ]] || continue
    readlink -f "$dev"
    return 0
  done

  # 2) Fallback: parse /proc/bus/input/devices
  awk '
    BEGIN { event="" ; iskbd=0 }
    /^H: Handlers=/ {
      event=""
      for (i = 1; i <= NF; i++) {
        if ($i ~ /^event[0-9]+$/) {
          event = "/dev/input/" $i
        }
      }
      if (iskbd && event != "") {
        print event
        exit
      }
    }
    /^B: EV=/ { next }
    /^$/ { event=""; iskbd=0 }
    /kbd/ { iskbd=1 }
  ' /proc/bus/input/devices | head -n1
}

need_cmd tmux

KEYBOARD_EVENT_DEVICE="$(find_keyboard_event_device)"
[[ -n "${KEYBOARD_EVENT_DEVICE:-}" ]] || die "Could not find keyboard event device"

log "Detected keyboard device: $KEYBOARD_EVENT_DEVICE"

if tmux has-session -t "$SESH" 2>/dev/null; then
  exec tmux attach -t "$SESH"
fi

log "Starting tmux session: $SESH"
tmux new-session -d -s "$SESH" -n main
tmux set-option -t "$SESH" -g mouse on
tmux set-option -t "$SESH" -g history-limit 200000

tmux split-window -t "$SESH":0 -h
tmux select-layout -t "$SESH":0 tiled

tmux_send "$SESH":0.0 "./unitree/install/bin/unitree_mujoco --network=$NET_IF"

tmux_send "$SESH":0.1 "sudo -E KEYBOARD_EVENT_DEVICE=$KEYBOARD_EVENT_DEVICE ./unitree/install/bin/g1_ctrl --network=$NET_IF"

tmux select-pane -t "$SESH":0.0
exec tmux attach -t "$SESH"