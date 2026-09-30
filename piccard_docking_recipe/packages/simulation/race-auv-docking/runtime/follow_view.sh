#!/usr/bin/env bash
# Point the Stonefish window's camera at the AUV for display recording (race-auv-docking/v1, --video only).
# Stonefish 1.5 has no scenario element for the GUI view: the trackball centre is only set from the window's VIEW
# panel. This replays that interaction with xdotool on the window, at window-relative pixels of the pinned GUI
# (1200x800 window, upstream quality), paced so the immediate-mode GUI renders between hover, press and release:
#   1. VIEW > Trackball center: one click on the down arrow selects the first robot, race_auv (scenario order).
#      The click is confirmed from the combo box itself (its dark text pixels: "Free" has far fewer than
#      "race_auv") and repeated only while the box still reads "Free", since a click before the scenario is
#      loaded is lost; a second click after a successful one would select race_station.
#   2. Right-drag (250,400)->(900,400) then (600,400)->(600,550): look north (the approach direction), ~22 deg down.
#   3. Seven wheel-up notches: orbit radius 5 m -> ~3.1 m, i.e. about 2.8 m behind and 1.1 m above the AUV.
#   4. Park the pointer in the corner (ffmpeg records without the cursor anyway).
#   5. Presentation view (#93): one [H] press hides the GUI panels and the status bar (Stonefish has no
#      panel-only toggle, so the frame then carries no clock; media.json maps video time to collector time). It is
#      confirmed from pixels (the status bar's static "Hit [K] for keymap" label stops correlating with its capture
#      taken while shown) and retried at most once, only while the bar is still seen. [H] is the only key ever
#      sent: ESC quits Stonefish and W/A/S/D/Q/Z move the camera. A failure leaves the HUD shown.
# The drag endpoints come from replaying OpenGLTrackball's arcball maths offline. Every step is logged to
# follow-view.json; a failure leaves the default view (or the HUD shown) and never affects the trial.
set -uo pipefail

log="$1"; step_seconds="${2:-2}"; timeout_seconds="${3:-120}"
steps=()
free_pixels=null; box_pixels=null; text_pixels=null; attempts=0
hud_status=not_attempted; hud_presses=0; hud_correlation=null; hud_state=unknown; hud_reference=""
record() { steps+=("{\"step\":\"$1\",\"wall_time_ns\":$(date +%s%N)}"); }
finish() {
  local joined
  joined=$(IFS=,; echo "${steps[*]:-}")
  printf '{"schema":"piccard.race-auv.follow-view/v1","status":"%s","window":"%s","step_seconds":%s,"glue_check":{"method":"VIEW trackball combo box pixels: drawn when >= 1500 cyan, selected when dark text pixels exceed 1.4x the Free reading","free_text_pixels":%s,"last_box_pixels":%s,"last_text_pixels":%s,"attempts":%s},"hud":{"status":"%s","presses":%s,"last_correlation":%s,"method":"one [H] press after the view is applied, confirmed when the status bar label no longer correlates (< 0.5) with its capture taken while shown; retried at most once"},"steps":[%s]}\n' \
    "$1" "${window:-}" "$step_seconds" "$free_pixels" "$box_pixels" "$text_pixels" "$attempts" \
    "$hud_status" "$hud_presses" "$hud_correlation" "$joined" > "$log"
  [[ -z "$hud_reference" ]] || rm -f "$hud_reference"
  exit 0
}
command -v xdotool >/dev/null || finish "xdotool_missing"
window=""
deadline=$((SECONDS + timeout_seconds))
while [[ -z "$window" ]] && ((SECONDS < deadline)); do
  window=$(xdotool search --onlyvisible --name '^Stonefish Simulator$' 2>/dev/null | head -1)
  [[ -n "$window" ]] || sleep 1
done
[[ -n "$window" ]] || finish "window_not_found"
record window_found
deadline=$((SECONDS + timeout_seconds))  # for drawing the box and confirming the glue
sleep "$step_seconds"
geometry=$(xdotool getwindowgeometry --shell "$window" 2>/dev/null | tr '\n' ' ')
[[ "$geometry" == *"WIDTH=1200"*"HEIGHT=800"* ]] || finish "unexpected_window_geometry"
move() { xdotool mousemove --window "$window" "$1" "$2"; }
click_left() {
  move "$1" "$2" || return 1; sleep "$step_seconds"
  xdotool mousedown 1 || return 1; sleep "$step_seconds"
  xdotool mouseup 1 || return 1; sleep "$step_seconds"
}
drag_right() {
  move "$1" "$2" || return 1; sleep "$step_seconds"
  xdotool mousedown 3 || return 1; sleep "$step_seconds"
  move $((($1 + $3) / 2)) $((($2 + $4) / 2)) || return 1; sleep 1
  move "$3" "$4" || return 1; sleep "$step_seconds"
  xdotool mouseup 3 || return 1; sleep "$step_seconds"
}
combo_pixels() {  # the combo box (window 20,532, 108x20): "<cyan box pixels> <dark text pixels>"
  local WINDOW X Y WIDTH HEIGHT SCREEN
  eval "$(xdotool getwindowgeometry --shell "$window")" || return 1
  ffmpeg -nostdin -v error -f x11grab -video_size 108x20 -i "${DISPLAY}+$((X + 20)),$((Y + 532))" -frames:v 1 \
    -f rawvideo -pix_fmt rgb24 - | python3 -c '
import sys
d = sys.stdin.buffer.read()
px = [d[i:i + 3] for i in range(0, len(d) - 2, 3)]
print(sum(p[0] < 60 and p[2] > 150 for p in px), sum(max(p) < 110 for p in px))'
}
read_combo() { local counts; counts=$(combo_pixels) || counts="0 0"; read -r box_pixels text_pixels <<<"$counts"; }
drawn() { ((box_pixels >= 1500)); }  # the cyan box fills ~2000 of the 2160 pixels once the GUI has drawn it
selected() { drawn && ((text_pixels * 10 > free_pixels * 14)); }  # "Free" has far fewer text pixels than a name
until read_combo; drawn; do
  ((SECONDS < deadline)) || finish "combo_not_drawn"
  sleep 1
done
free_pixels=$text_pixels
record combo_drawn
until selected; do
  ((SECONDS < deadline)) || finish "glue_not_confirmed"
  if drawn; then  # the box still reads "Free": the click was lost, or not made yet
    attempts=$((attempts + 1))
    click_left 138 542 || finish "glue_failed"
    move 1195 795 || finish "glue_failed"
  fi
  for _ in 1 2 3; do sleep "$step_seconds"; read_combo; selected && break; done  # let a slow frame land
done
record trackball_center_race_auv
drag_right 250 400 900 400 || finish "yaw_failed"
record yaw_north
drag_right 600 400 600 550 || finish "pitch_failed"
record pitch_down
move 600 400 || finish "zoom_failed"; sleep 1
for _ in 1 2 3 4 5 6 7; do xdotool click 4 || finish "zoom_failed"; sleep 1; done
record zoom_in
move 1195 795 || finish "park_failed"
hud_label() {  # "Hit [K] for keymap" in the status bar (window 1098,774, 80x22), grey, cursor not drawn
  local WINDOW X Y WIDTH HEIGHT SCREEN
  eval "$(xdotool getwindowgeometry --shell "$window")" || return 1
  ffmpeg -nostdin -v error -f x11grab -draw_mouse 0 -video_size 80x22 -i "${DISPLAY}+$((X + 1098)),$((Y + 774))" \
    -frames:v 1 -f rawvideo -pix_fmt gray -
}
read_hud() {  # correlation with the label captured while shown: ~1 while the status bar is drawn, ~0 once hidden
  local result
  result=$(hud_label | python3 -c '
import math, sys
ref, cur = open(sys.argv[1], "rb").read(), sys.stdin.buffer.read()
if not ref or len(cur) != len(ref):
    print("null unknown"); sys.exit()
ma, mb = sum(ref) / len(ref), sum(cur) / len(cur)
va, vb = sum((x - ma) ** 2 for x in ref), sum((y - mb) ** 2 for y in cur)
r = sum((x - ma) * (y - mb) for x, y in zip(ref, cur)) / math.sqrt(va * vb) if va and vb else 0.0
print(f"{r:.3f}", "shown" if r >= 0.5 else "hidden")' "$hud_reference") || result="null unknown"
  read -r hud_correlation hud_state <<<"$result"
}
toggle_hud() { xdotool key --window "$window" h; }  # the only key this script ever sends; ESC would quit Stonefish
hide_hud() {  # presentation view (#93): one [H] press hides the panels and the status bar; retried at most once
  hud_reference=$(mktemp) || { hud_status=capture_failed; return; }
  hud_label >"$hud_reference" 2>/dev/null || { hud_status=capture_failed; return; }
  python3 -c '
import math, sys
v = open(sys.argv[1], "rb").read()
m = sum(v) / len(v) if v else 0
sys.exit(0 if len(v) == 80 * 22 and math.sqrt(sum((x - m) ** 2 for x in v) / len(v)) >= 10 else 1)' "$hud_reference" ||
    { hud_status=status_bar_not_found; return; }
  local attempt
  for attempt in 1 2; do
    hud_presses=$attempt
    toggle_hud || { hud_status=key_failed; return; }
    for _ in 1 2 3; do  # let a slow frame land before deciding the press was lost
      sleep "$step_seconds"; read_hud
      [[ "$hud_state" == hidden ]] && { hud_status=hidden; record hud_hidden; return; }
      [[ "$hud_state" == shown ]] || { hud_status=unverified; return; }  # never press again without seeing the bar
    done
    record "hud_still_shown_after_press_$attempt"
  done
  hud_status=not_hidden
}
record pointer_parked
hide_hud
finish "applied"
