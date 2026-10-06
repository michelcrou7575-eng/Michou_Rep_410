# Glue Scale Weight Monitor / Pump Control — 410 Line

Reads a KILO TECK KWS CY300 glue-transfer scale via serial ASCII, brings it
into the S7-315-2 DP PLC through an Anybus Communicator (PROFIBUS gateway),
parses the ASCII weight into an INT (whole kg — the scale's decimal digit is
discarded, fractional precision isn't needed), and drives a glue fill pump
with a start/stop weight band plus latching over/underfill alarms.

## S7-300 STL hard limits (read before adding networks/labels)

Hit for real on 2026-10-02 (33 compile errors) — worth stating plainly so
it isn't rediscovered the same way again:

- **Jump labels (`JU`/`JC`/`JCN` targets) are 1-4 characters, letters and
  digits only — no underscores, no longer names.** This is a hard S7-300
  STL encoding limit (unlike IEC/SCL named labels), not a style choice.
  `RC_STP`, `PWOF_DN`, etc. all failed to compile for this reason.
- **A FUNCTION/DATA_BLOCK's title/header comment block has a hard total
  size limit.** FC155's accumulated changelog grew to ~290 lines and
  STEP7 failed with "Byte offset/number too big" right at the VERSION
  line. Keep the header's changelog short (a few lines per version, or a
  condensed multi-version summary like the one below) — full history
  lives in git log, not in the compiled source's comment block.
- **Individual network titles and inline comments are also length-capped**
  (STEP7 just silently truncates past the limit — a "Comment or title
  length too big" warning, not a fatal error, but still lossy).
- **An `S`/`R`/`SD`/`SS`/etc. "coil" instruction right after a label that's
  reached via a jump (`JC`/`JCN`/`JU`) can't trust RLO to be 1** — put an
  explicit `SET;` before it. This file's own pump-control network already
  did this correctly (`TRXF:`/`CLOS:` both have `SET;` before their
  `S`/`R`); the v0.22-v0.24 power-sequence networks (`RCS1`/`RCS2`/`RCS3`/
  `RCS5`) initially didn't, and Michou hit it for real ("ReCalib_Step goes
  to value 3 and stuck") — fixed in v0.25. A plain `A`/`O`/etc. bit-logic
  instruction right after a label doesn't have this problem — it loads
  fresh regardless of what RLO was before the jump.
- **`SD` is the NON-retentive on-delay timer mnemonic (`S_ODT`), not the
  stored one** — a mistake repeated throughout this project's comments
  all session ("the stored timer counts down untouched by repeat calls"
  is wrong). The actual retentive/stored on-delay is `SS` (`S_ODTS`).
  `SD`'s count resets to 0 the instant its enable input goes false, so a
  timer meant to keep running after only a one-scan trigger (like the
  old underfill settle timer) silently never reaches its preset.
- **This project now uses IEC timers instead of classical S5 timers**
  (v0.26/v0.27, per Michou: "Timers don't start!!! What about using
  SFB4 instead?"). Critical difference from classical timers: **an IEC
  timer is call-driven, not hardware-autonomous.** A classical S5
  timer, once started, keeps counting in dedicated CPU hardware whether
  or not the program revisits the starting instruction on later scans.
  An IEC timer does NOT — it only advances its elapsed time while its
  own `CALL` instruction actually executes that scan with `IN=1`. A
  "start once in one step, check the result two steps later" pattern
  (the old T53, the 5s K1 hold) silently stalls forever with a naive
  swap, because the timer is simply never called again once the state
  machine moves to a different step. Fix: every timer's `IN` condition
  is recomputed and the timer(s) `CALL`ed **unconditionally, every
  single scan**, regardless of which step is currently active — see
  "Power-up calibration workaround" below.
- **SFB2/3/4 (the firmware-native IEC timer system function blocks) are
  NOT available on every S7-300 CPU** — v0.26 tried `SFB4` directly and
  it failed to compile ("Symbol PT not found in symbol table", "FB
  requires the specification of an instance DB") on this CPU (315-2
  DP). The fix (v0.27) was importing the **library** `TON` Function
  Block from STEP7's Standard Library ("IEC Function Blocks") into the
  project instead — an ordinary user FB with its own project-assigned
  number, not a fixed SFB. If a future S7-300 target genuinely has
  native SFB2/3/4, this project doesn't need them — the library FB
  works everywhere.

## Hardware / signal chain

```
KWS CY300 scale --RS232 (38400 8N1, free-running ASCII)--> Anybus Communicator
Anybus Communicator --PROFIBUS DP (node addr 4)--> Siemens S7-315-2 DP CPU
```

- Scale streams unsolicited ASCII lines continuously, e.g. `GROSS:       0.0kg`.
- Anybus: Custom Produce/Consume protocol, Start char 0x0A, End char 0x0D,
  Inter-telegram timeout = Default (3.5 characters).
- Transaction template "GROSS FILTER" (Consume):
  - Constant field, 6 bytes = ASCII "GROSS:" (`71,82,79,83,83,58`) — filters
    out any other line (NET, blank separator).
  - Variable data field, min 8 / max 20 bytes, Subnet delimiter = End
    pattern (13 = 0x0D), Fill padding ON (value 0).
- PROFIBUS: Anybus Communicator GSD (Ident 0x183B), modular slot-based —
  Slot 3 = "4 bytes input module", Slot 5 = "16 bytes input module", all
  other slots Empty. Address range: IB272-287 (16 bytes) + IB289-292
  (4 bytes) — non-contiguous, IB288 is an unused gap.
- PLC reads both blocks via `CALL SFC 14 (DPRD_DAT)` (consistent DP read),
  not `L IB`/`L PIB` — required because IB272-292 falls outside this CPU's
  128-byte process image (I0.0–I127.7).

## ASCII payload layout (DB4 "ABC3000A_DB")

| Byte(s) | Content |
|---|---|
| 0-6 | Integer part, right-justified, space-padded, up to 3 digits used (0-300kg scale) |
| 7 | `.` (0x2E) |
| 8 | Decimal digit (parsed, then discarded — see below) |
| 9-10 | `k`,`g` |
| 11-19 | Padding (0x00 / spaces) |

FC155 parses Byte_0-6 into `"GLUE SCALE CONTROL DB".GrossWeight_Actual : INT`
(whole kg). Byte_8 is no longer converted into a fractional part — the scale
transaction doesn't need sub-kg precision. Byte_7/9/10 are checked against
their fixed values each scan; any mismatch (or a non-zero SFC14 RET_VAL) sets
`ParseError` and holds the last-good weight/pump/alarm state rather than
acting on a bad telegram.

## Control logic ("GLUE SCALE CONTROL DB" / DB105)

- **Pump_ON** (was `Valve_Open` — renamed, since it's always driven a pump,
  never a valve): turns on when `GrossWeight_Actual < Fill_Start_SP`, off at
  `GrossWeight_Actual >= Fill_Stop_SP`. (`Hysteresis` is still declared in
  DB105 for struct-layout compatibility but is no longer read.) On the rising
  edge of turning on, snapshots the current weight into
  `GrossWeight_Actual_Mem` — see "Transfered weight" below. Mirrored onto the
  real actuator bit, `"HMI DB".Glue_Fill_Pump_ON` — see HMI bridge section.
- **Alarm_Overfill**: latching, sets if `GrossWeight_Actual > AlarmLimit_Overfill_SP`.
- **Alarm_Underfill**: 5 seconds after Pump_ON falls (via FB555
  "IEC TIMERS"'s `UNDERFILL_IN`/`UNDERFILL_OUT` — `IN` fed directly
  from `NOT Pump_ON`, no edge-latch needed), checks `GrossWeight_Actual < AlarmLimit_Underfill_SP`
  continuously from that point on while the pump stays off; latching.
  **Also now latches during an active fill** (v0.36, Michou 2026-10-06:
  "Underfill Alarm should go off even when pump is running! Could be a
  leak!") — see "Leak detection (v0.36)" below for the two new networks
  that reuse this same alarm/interlock while `Pump_ON` is true.
- **Alarm_ScaleFault**: latching, sets if either:
  - `GrossWeight_Actual` matches `NegativeUnderScore_Value` (INT, -20663)
    — confirmed by Michou: the real INT value the KWS CY300 gives when
    its reading goes negative (supersedes an earlier, incorrect
    5222222/DINT assumption from before this was confirmed against the
    real scale). Runs on the already-stored INT, no truncation concern
    since -20663 fits fine in 16-bit range; or
  - `GrossWeight_Actual < PlatformMinWeight_SP` (30kg) — the empty
    platform's own dead weight, so a GROSS reading below that (0, or
    any other corrupted negative value — the Byte_0-5 parse loop
    doesn't validate digit range, so a non-digit byte in a garbled
    telegram can genuinely drive the parsed integer to something other
    than -20663) means the reading can't be trusted, not a real
    underfill.

  Also gates pump/alarm updates the same way ParseError does (so the
  pump doesn't react to a bad reading as if it were a real weight), and
  additionally forces `Pump_ON` off immediately and unconditionally —
  unlike ParseError, which only freezes the pump at its last commanded
  state, a latched Alarm_ScaleFault is an active stop ("No Pump work!").
- **Overfill/Underfill interlock**: while `Alarm_Overfill` OR
  `Alarm_Underfill` OR `Alarm_PossibleLeak` (v0.36) is latched, `Pump_ON`
  is forced off and held off (blocks automatic re-engagement) until
  `Reset_Alarms` clears the alarm.
- **Reset_Alarms**: HMI-driven input bit. While TRUE, clears
  `Alarm_Overfill`, `Alarm_Underfill`, `Alarm_ScaleFault` and (v0.36)
  `Alarm_PossibleLeak` together every scan (level-conditioned, not edge
  — safe as a momentary acknowledge button, since any alarm whose fault
  condition is still present just re-latches the next scan). Runs
  unconditionally, even on a scan where ParseError/Alarm_ScaleFault
  would otherwise skip the rest of the block, so a latched fault can
  always be cleared. The PLC side is wired; connecting an actual HMI
  button to this bit is still open.

## Leak detection (v0.36, FB555 v0.7, DB105 v0.20)

Per Michou (2026-10-06): *"Underfill Alarm should go off even when pump
is running! Could be a leak! Should stop the Pump like overfilled! ...
there should be a timing alarm too: If Fill take too long or if the
glue Barrel (Container on the scale) is emptying too fast ... could be
a leak also. We need to forsee any/every scenario that could cause a
disaster (Glue spill!!!)."* Three new networks, right after "Pump
control":

- **Fill stall detection** — while `Pump_ON`, the weight should be
  rising. `FillStall_Ref` (DB105, internal) reseeds to the current
  weight every time `Pump_ON` has a rising edge (same place
  `GrossWeight_Actual_Mem` already snapshots) and again every time at
  least `FillStallMinRise_SP` kg of progress is detected — this
  naturally resets a watchdog timer (`FB555`'s `TON_FILLSTALL`) each
  time real progress happens, so it only accumulates during a genuine
  stall. If 60s (placeholder, hardcoded in FC155's "Timers" like every
  other timer duration in this file) passes with no progress, it sets
  **`Alarm_Underfill`** — reused, not a new alarm identity, per
  Michou's own wording — which already interlocks `Pump_ON` off via the
  network above, same as overfill.
- **Fill timeout** — complements the stall check: `Pump_ON` continuously
  for 5 minutes (placeholder), regardless of whether some progress is
  still happening, also sets `Alarm_Underfill`. Catches an abnormally
  slow fill that never quite triggers the stall check (weight keeps
  inching up, just far slower than normal).
- **Possible leak** — a genuinely different scenario, running during
  *normal operation* (`Pump_ON` false), not during a fill: every 30s
  (placeholder) while idle, if weight has dropped more than
  `MaxDrainDrop_SP` kg (5kg placeholder) since the last check, sets a
  **new** alarm, `Alarm_PossibleLeak` (not reused `Alarm_Underfill` —
  stopping the fill pump wouldn't do anything, it isn't running).
  Latches, interlocks `Pump_ON` off, cleared by `Reset_Alarms`, drives
  the RED panel lamp like the other three alarms.

**All thresholds/durations above are placeholders** — this repo has no
way to know the real fill rate or normal idle glue-consumption rate.
`FillStallMinRise_SP`/`MaxDrainDrop_SP` are HMI-writable DB105 setpoints
(tune from there); the three new `TIME` literals live in FC155's
"Timers" network (hardcoded, like every other timer duration in this
file — none of those are HMI-adjustable either, kept consistent).

**Not implemented, confirmed deferred (Michou, 2026-10-06)**: Michou's
closing line — *"could cause ... machine feed to interrupt"* — asked
for `Alarm_PossibleLeak` to also stop whatever downstream process is
consuming the glue, not just block the (unrelated, already-stopped)
fill pump. Asked directly: detect-and-alarm only, for now - no
cross-subsystem interlock to the Rotaliner/Tuber process. Revisit if a
specific output/mechanism for that is identified later.

## Transfered weight

`"HMI DB".Actual_Transfered_Weight` = `GrossWeight_Actual` −
`GrossWeight_Actual_Mem`, recomputed every scan. `GrossWeight_Actual_Mem` is
a snapshot of the weight taken once, on the scan `Pump_ON` is newly
commanded on — i.e. this tracks how much has been transferred since the
pump last started, confirmed against the real pump-start event, not a
free-running or reset-driven total.

## Manual test and indicator lamps (all in "OUTPUTS DB" / DB60, shared
## with other subsystems — see `DB60_OUTPUTS_DB.awl`)

**As of FC155 v0.29, FC155 no longer drives these outputs directly.** The
`Glue_Fill_Pump_Test` and alarm-lamp behavior described below is still true
end-to-end, but it's now implemented as FC155 writing a few command bits in
`"PANEL LED CMD DB"` (DB107) and calling `FC158 "SCALE PANEL LAMPS"`, a
separate, reusable panel-lamp driver built for Michou's "Panel LED drive"
request. **See the "Panel LED drive (FC158 / DB107 / FB556 / DB556)" section
below for the full design** — this section just covers what FC155 itself
still decides:

- **Glue_Fill_Pump_Test** (`"HMI DB"`, mirrored from the real panel switch
  `"INPUTS DB".WHT_SW_1_Pump_Test` as of v0.31 — see "Power-up
  stabilization" below): lights **`WHT_SWL_1_Pnl_Pump_Test` only** (sets
  DB107's `WHT_1` true, `WHT_2`/`WHT_3` stay off, scan bits false —
  FC158's "independent" mode) in place of the scan pattern below. Runs
  unconditionally (not gated by ParseError/Alarm_ScaleFault), so manual
  test still works during a scale fault. Does **not** override
  `Glue_Fill_Pump_ON` here — see "Drive physical glue pump output"
  network, unchanged.
  **v0.31 wrongly called this a bug** ("all three lamps should light
  for the test") and briefly fixed it that way; **v0.35 (Michou,
  2026-10-06) corrected the record** - WHT_1-only is intentional, each
  white lamp mirrors one specific switch's own function (confirmed by
  DB60/DB61's own renames), so lighting Calib/Alarm_Reset's lamps during
  a pump test would be misleading. The v0.31 "bug" framing is left in
  place below only as an accurate record of what was believed at the
  time, before the per-lamp meanings were known - not rewritten.
- **The three white lamps/switches have real names now** (DB60/DB61 v0.5/
  v0.2, from Michou's own re-export, 2026-10-06): each white panel lamp
  mirrors its own matching panel switch — `WHT_SWL_1_Pnl_Pump_Test` /
  `WHT_SW_1_Pump_Test`, `WHT_SWL_2_Pnl_Calib` / `WHT_SW_2_ReCalib`,
  `WHT_SWL_3_Pnl_Alrm_Ack` / `WHT_SW_3_Alarm_Reset`. Confirms what these
  were for — not just a generic 3-lamp scan, but Pump Test / Recalibrate /
  Alarm Reset indicators.
- **WHT_SWL scan pattern**: while `Pump_ON` is true (and Test is not
  active), FC155 sets DB107's `WHT_ScanFwd` true, which makes FC158 cycle
  the three lamps 1→2→3→1… as a running-light "pump active" indicator
  (500ms/step, FC158's own clock — see below). Idle (`Pump_ON` false, Test
  not active): all scan/independent bits cleared, lamps off.
  **Not bench-verified** — hand-traced, not confirmed on real hardware.
- **WHT_SWL_2_Pnl_Calib blinks during recalibration** (v0.33/v0.34,
  Michou 2026-10-06): a "Recalibration indicator" network drives DB107's
  `WHT_2` from `"M 6.5"` (a clock-memory bit — blinks rather than solid
  on) whenever `ReCalib_Step<>0` (manual or auto-triggered), overriding
  the scan-pattern network above. No explicit off-reset needed, since
  `Pump_ON` is always false while recalibrating (enforced by "Power-up/
  recalibration interlock" below) so the override network's own `CLR`
  already zeroes `WHT_2` first every scan this doesn't fire. "Power-on +
  calibrate sequence" (below) also `SET`s `WHT_2` once, the exact scan a
  recalibration is triggered — closes the one-scan gap before
  `ReCalib_Step` itself updates and this network's own blink takes over.
  **Not confirmed**: whether `M 6.5` is actually configured as a clock
  memory bit in HW config — if not, this just reads as a static value
  rather than blinking.
- **RED_LED_Pnl**: as of Michou's own v0.31 edit, **blinks** (via DB107's
  `RED_Blink`, not solid `RED_On` anymore) if `Alarm_Overfill` OR
  `Alarm_Underfill` OR `Alarm_ScaleFault` is latched — more attention-
  grabbing than solid. **GRN_LED_Pnl**: solid on (unchanged, `GRN_On`) if
  none are. (Michou's original spec named only Overfill/Underfill for RED
  and "no alarm" for GRN — `Alarm_ScaleFault` was folded into both here so
  GRN's "no alarm" claim stays accurate; flag if ScaleFault shouldn't be
  included.)

## Panel LED drive (FC158 / DB107 / FB556 / DB556)

New, separate, reusable subsystem per Michou's request (2026-10-05): "a
separate 'Panel LED drive' Function ... Controlled by a Byte or Array of
bool to: Scan 3 WHT LEds FWD/REV, each Independant or Panic Mode (Any ways
to impress), And GRN Solid/blink, RED Solid/blink." Drives the same 5 lamp
outputs FC155 used to drive directly (`GRN_LED_Pnl`, `RED_LED_Pnl`,
`WHT_SWL_1_Pnl_Pump_Test`/`WHT_SWL_2_Pnl_Calib`/`WHT_SWL_3_Pnl_Alrm_Ack`,
all in `"OUTPUTS DB"`/DB60), now from a command interface any caller can
write.

**Built as FC157, renumbered FC158 by Michou** (2026-10-06, his own "Add
FC158 PANEL LED DRIVE" / "Refactor glue scale logic" commits) — FC157
wasn't actually free on the live project, confirming the "not
independently verified" caution flagged since this was first built. He
also renamed the FUNCTION itself from "PANEL LED DRIVE" to **"SCALE PANEL
LAMPS"**, and renamed FB556/DB556's own symbols to use spaces rather than
underscores (`"PANEL LED TIMERS"` / `"PANEL LED TIMERS DB"`), matching his
convention for every other named block in the project. All reconciled
here - rename only, no behavior change from what's described below.

- **`"PANEL LED CMD DB"` (DB107)** — the command/status interface. Named
  BOOL command bits rather than a packed integer "mode byte": S7 already
  packs consecutive BOOLs into bytes, so this gets the same compact
  footprint with named-bit clarity and no decode logic needed. Fields:
  `WHT_ScanFwd`, `WHT_ScanRev`, `WHT_Panic`, `WHT_1`/`WHT_2`/`WHT_3`
  (independent direct drive), `GRN_On`, `GRN_Blink`, `RED_On`, `RED_Blink`,
  plus internal `WHT_ScanStep` and 6 spare bits for future growth. Michou
  kept every field name exactly as proposed when he typed this block in.
- **`FC158 "SCALE PANEL LAMPS"`** — stateless logic, reads DB107 + FB556's
  clocks, drives the 5 DB60 outputs. White-lamp priority (first match
  wins): **1. `WHT_Panic`** — all 3 lamps strobe together, fast (150ms) —
  **2. `WHT_ScanFwd`/`WHT_ScanRev`** — 1→2→3→1 or 3→2→1→3, 500ms/step —
  **3. otherwise** — "independent": `WHT_1`/`WHT_2`/`WHT_3` mirrored
  straight to the outputs. GRN/RED: `*_Blink` overrides `*_On`/solid;
  neither bit means off.
- **`FB556 "PANEL LED TIMERS"` / `"PANEL LED TIMERS DB"` (DB556)** — same
  multi-instance `"TON"` wrapper pattern as FB555/DB555 (see the hard-
  limits section above for why), called once per scan from FC158. Two
  live clocks now: SCAN (500ms, white-lamp scan pacing), PANIC (150ms,
  deliberately faster/"impressive"). A third, ALARM, paced GRN/RED blink
  until v0.4 (2026-10-06) — its self-oscillating pattern only held Q
  true for one scan every 500ms (~2% duty cycle: a brief flash, not a
  blink — Michou: "the RED Lamp is pulsing when Fault! Need Blinking!").
  GRN/RED blink now reads `"M 6.5"` directly (a clock-memory bit, same
  one used for the Calib-lamp blink — a real 50/50 square wave).
  `TON_ALARM`/`ALARM_OUT` are left declared in FB556, unused.
- **Who calls it**: FC155 sets `WHT_ScanFwd`/`WHT_1` (lamp scan while
  pumping, `WHT_1` only for `Glue_Fill_Pump_Test` — see "Manual test and
  indicator lamps" above) and `GRN_On`/`RED_Blink` (alarm state), then
  `CALL`s FC158 itself — no OB1 edit needed, FC158 just rides FC155's
  existing scan-cycle call. FC155 never touches `WHT_Panic`/`GRN_Blink`/
  `RED_On`, so those stay free for independent HMI/manual/test-table
  control without FC155 fighting them.
- **Not yet done**: no HMI control wired to `WHT_Panic`/`GRN_Blink` yet —
  Michou asked for the capability, not a specific trigger; needs a
  screen/button decision. Not bench-verified (compiles cleanly against
  this repo's own cross-checks, not run on real hardware). FB556/DB556
  block numbers are only confirmed free within this repo's own limited
  visibility, same open risk as every other block number picked this
  project (see "Still open") — FC157→FC158 is a concrete example of that
  risk actually materializing.
- **`Pump_Out`** (`"OUTPUTS DB"`, was `Output_060`, briefly `Pump_Out_
  Valve`) — Michou's original "Panel LED drive" message named this bit
  too, but it's a valve, not a lamp, and nothing about its control
  semantics was specified. Renamed in DB60 only (Michou's own v0.5
  re-export shortened the name further); **not** part of FC158's scope
  and not driven by any logic in this repo yet.

## HMI bridge ("HMI DB" / DB10)

FC155 mirrors these fields to/from `"HMI DB"` (DB10) every scan,
unconditionally (not gated by ParseError/Alarm_ScaleFault) unless noted:

- `Fill_Start_Weight_SP` → `Fill_Start_SP`, `Fill_Stop_Weight_SP` →
  `Fill_Stop_SP`, `AlarmLimit_Overfill_SP` → `AlarmLimit_Overfill_SP`,
  `AlarmLimit_Underfill_SP` → `AlarmLimit_Underfill_SP` — operator-entered
  setpoints flow HMI → scale, one-way (the touch panel's own numeric-entry
  widget is the display of record for what was last typed).
- `GrossWeight_Actual` → `Actual_Glue_Weight` — live reading, scale → HMI.
- `GrossWeight_Actual − GrossWeight_Actual_Mem` → `Actual_Transfered_Weight`
  — see "Transfered weight" above.
- `"RUN_DATA_DB".Web_Velocity_Filtered` (410 seam-AGC subsystem, DB640,
  owned by FC120/FB610) → `Bottomer_Velocity` — this glue-scale block and
  the seam monitor share the same physical tube, so the same web velocity
  applies to both; not this block's own measurement.
- `Pump_ON` OR `Glue_Fill_Pump_Test` → `Glue_Fill_Pump_ON` — the real pump
  output. Runs unconditionally (see "Manual test and indicator lamps"
  above) — found by checking what, if anything, drove a real output from
  the pump decision: nothing did, anywhere in this project, until this was
  wired.

`"HMI DB"` was restructured directly online (HMI DB v0.3): `Data_Write`/
`Data_Read` (WORD, confirmed unused anywhere) were removed, the three
separate `Spare_INT_121/122/123` collapsed to one `Spare_INT`, and bools
reordered. Since FC155/FC160 reference every field by symbol name (not raw
offset), this is safe as long as both are recompiled together — already
true, both were uploaded from the live, working PLC project.

## Power-up calibration workaround (KWS CY300 defect, FC155 / DB105 / "OUTPUTS DB")

The KWS CY300 resets its zero/calibration reference on every power-up —
a known hardware defect. Michou's workaround adds two relays, wired per
his hand-drawn diagram (2026-10-01), onto two previously-unused bits of
`"OUTPUTS DB"` (DB60 — the same shared output DB this block already uses
for GRN_LED/RED_LED/WHT_SWL_1/2/3, now tracked in full as
`DB60_OUTPUTS_DB.awl`). This logic lives directly in FC155 (folded in
from a short-lived separate FC156, per Michou: "FC155 containes your
FC156"):

- **K1 "Calibration Relay"** — switches the load-cell S+/S- signal pair
  through a shunt-cal loop built into the scale. Coil on PLC terminal 40,
  `"OUTPUTS DB".KILOTECK_Calib_K1` (DB60.DBX7.6, was the placeholder
  `Output_061`).
- **K2 "Power Sw relay"** — bridges the scale's remote ON/OFF terminal to
  its Com terminal, i.e. energizing it is equivalent to pressing the
  scale's own power button. Coil on PLC terminal 39,
  `"OUTPUTS DB".KILOTECK_Power_K2` (DB60.DBX7.7, was the placeholder
  `Output_062`).

K2 is a momentary button-press simulation, not a plain on/off.

FC155 runs timing via **FB555 "IEC TIMERS"** (Michou's design,
2026-10-03, renamed from "IEC_TIMERS" in his own live edit, 2026-10-06 —
see its own changelog) — one wrapper FB holding all 8 timers as
multi-instance `"TON"` (library FB) children, called **once per scan**
with one instance DB, `"IEC TIMERS DB"` (DB555). See `FB555_IEC_TIMERS.
awl` and "S7-300 STL hard limits" above for why a library FB instead of
SFB4. Sequencing state lives in DB105 ("GLUE SCALE CONTROL DB" — this
subsystem's own state, not the shared output DB), specifically
`ReCalib_Step`.

**"Process ON"** is the core building block (Michou, 2026-10-03,
correcting an earlier wrong design where K1 started before K2): **K1
and K2 start simultaneously** — K2 pulses 0.5s
(`RC_K2ON_IN`/`RC_K2ON_OUT`), K1 stays on for a flat **8s** (changed from
5s by Michou, 2026-10-06 — `RC_K1HOLD_IN`/`RC_K1HOLD_OUT`, started the
same step as K2's pulse). Two things trigger it:

- **Full recalibration** (`Scale_ReCalib_Req` rising edge, detected via
  raw symbol `"FP 201.1"`, ignored while already running):
  recalibrating needs a full power cycle first, so this sequence is
  `ReCalib_Step` 1→5 — **1**: K2 **alone** (no K1) for **5s**, power
  off (`RC_K2OFF_IN`/`RC_K2OFF_OUT`); **2**: a 3s dwell with nothing
  active (repurposes FB555's `PWRON_IN`/`PWRON_IN_TIME`/`PWRON_OUT`
  slot — see below); **3**: Process ON starts (K1+K2 together) — also
  clears `Scale_ReCalib_Req` here now (Michou, 2026-10-06) rather than
  waiting for full completion; **4**: waiting for K1's 8s hold (started
  in step 3) to elapse; **5**: K1 releases, done, `Scale_Calibrated` set
  (see "Power-up stabilization" below). Once running, every one of
  FB555's 8 `IN` parameters is recomputed and the whole FB `CALL`ed
  every scan regardless of which step is active (see "Timers" network)
  — this is the part a classical-timer "fire once, check back later"
  design can't do.
- **Standalone manual power-on** (`Scale_PowerON_Pulse`, ignored while
  `ReCalib_Step` is mid-sequence): enters the *same* state machine
  directly at step 3 — "run Process ON standalone," skipping the
  power-off+wait. Auto-clears `Scale_PowerON_Pulse` when step 5
  completes, same self-clearing idiom as the manual pulses below.
- **`Scale_ReCalib_Req` itself** is driven two ways (v0.31): the panel
  switch `"INPUTS DB".WHT_SW_2_ReCalib` `SET`s it on its own rising edge
  (`"FP 201.4"`, **not** a level-mirror — step 3 above explicitly clears
  it, so a continuously-held switch can't just re-assert it right back),
  or FB555's new `STARTUP_OUT` auto-triggers it 30s after every restart
  (see "Power-up stabilization" below). Not yet confirmed by Michou that
  `WHT_SW_2_ReCalib` is meant for this - inferred from its name alongside
  his own `WHT_SW_1_Pump_Test` wiring.

**Manual power-off pulse** (`Scale_PowerOFF_Pulse`, ignored while
`ReCalib_Step` is mid-sequence) is fully independent of all of the
above: a single 3s K2 pulse (`PWROFF_IN`/`PWROFF_OUT`), no K1 involved,
no step counter — gated directly on the bit's own level; auto-clears
when the pulse completes. Michou didn't ask for this one to change, so
it's untouched by the "Process ON" correction.

- **Auto-trigger on restart** (new, Michou 2026-10-06): a third trigger,
  `"IEC TIMERS DB".STARTUP_OUT`'s own rising edge (`"FP 201.3"`),
  ORed into the same recalibration entry check as `Scale_ReCalib_Req`
  (both edges computed independently into a new `#RC_TRIG` temp, then
  ORed - see "Power-up stabilization" below for why). This is what
  "restarts the scale and makes sure it's calibrated" automatically
  after a power outage, with no HMI button needed.

Every sequence guards against starting while another is mid-run, so
K1/K2 are never driven by more than one action in the same scan.
FB555's other two timers, `LAMP_IN`/`LAMP_OUT` and
`UNDERFILL_IN`/`UNDERFILL_OUT`, cover the lamp-scan flasher and the
underfill settle timer respectively — see "Manual test and indicator
lamps" and the underfill alarm in "Control logic" above. FB555 and
DB555 are a best guess for free block numbers, confirmed only within
this repo's own tracked files — **not independently verified against
the live project's full block list**, same unresolved caution as every
block number picked this session (and the one that already bit FC157,
renumbered FC158 - see "Panel LED drive" above).

### Power-up stabilization (FC155 v0.32 / FB555 v0.6 / DB105 v0.19 / OB100 v0.2)

Per Michou (2026-10-06): *"need a 30 sec delay before restarting after
power has resumed ... Need to restart KiloTech Scale too and make sure
it is calibrated before operating the pump!"* An OB can't literally pause
(that trips the CPU's scan-time watchdog), so the 30s wait and
auto-recalibration are implemented as ordinary cyclic logic in
FC155/FB555, armed by two bits OB100 resets every restart:

- **`PowerUp_Settled`** (DB105) — an internal one-shot latch: FALSE for
  exactly the first OB1 scan after OB100 resets it, TRUE forever after
  (FC155's "Timers" network reads it into `#STARTUP_IN`, then
  immediately `SET`s it true for every later scan). Feeding that into
  FB555's new `STARTUP_IN`/`STARTUP_IN_TIME`(30s)/`STARTUP_OUT` TON
  gives a timer that reliably measures "30s since the most recent
  restart," not a stale carryover — DB555's own actual values are
  battery/cap-backed too, so without the one-scan FALSE pulse this
  timer could already read done on a fresh power-up.
- **`STARTUP_OUT`**'s rising edge auto-triggers a **full recalibration**
  (same state machine as a manual `Scale_ReCalib_Req` pulse — power K2
  alone off 5s, 3s dwell, Process ON, 5s hold, done) exactly once per
  restart. This is the "restart the scale" part: recalibrating already
  starts with a scale power-cycle, so no separate scale-restart step was
  needed.
- **`Scale_Calibrated`** (DB105) — reset FALSE by OB100, `SET` by FC155
  only when the recalibration sequence reaches done (`RCS5`). The new
  **"Power-up/recalibration interlock"** network blocks `Pump_ON` while
  `Scale_Calibrated` is FALSE *or* `ReCalib_Step` is non-zero (a
  recalibration — auto or manual — is actively running) — so the pump
  physically cannot start until the 30s stabilization wait **and** the
  full recalibration sequence have both completed. Total worst-case
  delay after a power outage: ~30s wait + ~16s recalibration (5s power-
  off + 3s dwell + 8s K1 hold, which starts at the same step K2's 0.5s
  pulse does and so already contains it - not additive on top, plus the
  scan time each step takes to notice) ≈ 46s before the pump can run
  again. (8s per Michou's 2026-10-06 change to `RC_K1HOLD_IN_TIME` - see
  "Power-up calibration workaround" above.)
- Operator setpoints are untouched by any of this — only state/status
  bits are reset. `Scale_Calibrated` is **not** reset at the start of a
  later manual recalibration (only by OB100) — once true it stays true
  until the next restart, since `ReCalib_Step<>0` already blocks the
  pump for the sequence's own duration regardless.

**Still open / flagged for confirmation**:
- **Manual retyping into STEP7 reintroduced a real bug once** (2026-10-06
  report: "ReCalib_Step get stuck! FC155 not working!") - a parameter
  swap in the "Timers" CALL (`RC_K2OFF_IN`/`RC_K1HOLD_IN` crossed) -
  fixed by Michou in his next update, confirming the diagnosis. Worth
  downloading/importing this file directly from the repo rather than
  retyping it, if the live project's workflow allows that - retyping
  risk is real, even if the `WHT_1`-only `WTST` behavior chased in
  v0.31/v0.35 turned out not to be a bug at all (see "Manual test and
  indicator lamps" above).
- **OB100 already exists on the live PLC** (confirmed by Michou,
  2026-10-06) — `OB100_COMPLETE_RESTART.awl` is NOT something to
  download as a replacement block. Its networks (now including the
  `PowerUp_Settled`/`Scale_Calibrated` resets) need to be pasted into
  the end of the real, existing OB100.
- The 30s stabilization figure and the recalibration's own ~16s are
  both per Michou's stated durations - neither has been bench-verified
  against an actual power-outage/recovery test yet.
- `Scale_PowerOFF_Pulse` is wired to nothing yet. `Scale_ReCalib_Req`
  and `Reset_Alarms` are now wired to `WHT_SW_2_ReCalib`/`WHT_SW_3_
  Alarm_Reset` (v0.31, my own inference from Michou's own `WHT_SW_1_
  Pump_Test` → `Glue_Fill_Pump_Test` wiring pattern) — **not confirmed
  by Michou**, flag if those two switches are meant for something else.
- Block numbers picked in this repo are only a best guess against this
  repo's own limited visibility, confirmed **not** reliable in practice
  now: FC157 "PANEL LED DRIVE" was renumbered FC158 "SCALE PANEL LAMPS"
  by Michou because FC157 wasn't actually free on the live project.
  FB556/DB107/DB556/FB555/DB555 remain unconfirmed the same way.
- `_Pnl` suffix: confirmed real for all five lamp outputs
  (`RED_LED_Pnl`/`GRN_LED_Pnl` by a real compile error; `WHT_SWL_1/2/3_
  Pnl` by Michou's working online version) — and since refined further
  (v0.5, 2026-10-06) to `WHT_SWL_1_Pnl_Pump_Test`/`_2_Pnl_Calib`/`_3_
  Pnl_Alrm_Ack`, matching the real switch each lamp mirrors.
- The "Process ON" sequence (v0.28) is a fresh correction, not yet
  confirmed on real hardware against Michou's exact intent — in
  particular, whether `Scale_PowerOFF_Pulse`'s standalone 3s power-off
  duration should also become 5s to match the recalibration's power-off
  step (Michou's message specifically said "K2 alone 5sec" in the
  recalibration context; the standalone manual power-off wasn't
  mentioned, so it was left at 3s — flag if that's wrong).
- The IEC-timer migration (v0.26/v0.27, FB555) overall is still not
  bench-verified beyond "it compiles and the PLC runs it" — confirm
  the self-oscillating lamp-scan flasher's actual timing on real
  hardware.
- FB555 needs confirming as a genuinely free block number on the live
  PLC before download — this repo can't see the whole project. DB555 is
  no longer a guess, though — Michou explicitly wants the instance DB
  numbered to match its FB (DB555 for FB555, DB556 for FB556, 2026-10-06),
  so that part's settled by his own instruction, not a pick of ours.
- The "Update" network (`#UPDATE` := rising edge of `"M 6.7"`) now gates
  "Detect scale fault" and "Overfill alarm" (and "Platform weight sanity
  check" together with `Pump_ON`) behind `Pump_ON OR #UPDATE` instead of
  running every scan — adopted from Michou's own online edit, but
  **not independently confirmed as intentional**: if `M 6.7` only
  pulses rarely, this means the scale-fault and overfill checks only
  evaluate occasionally rather than continuously. Worth confirming given
  these are safety-relevant checks.

## Files

- `FC155 GLUE SCALE LOGIC` — canonical source, version 0.36. SFC14 reads,
  ASCII parse (INT weight, decimal digit discarded), pump control with
  overfill/underfill interlock, platform-weight sanity check + scale-fault
  pump interlock, alarm latching and reset, HMI DB bridge (setpoints, live
  weight, transfered-weight calc, bottomer-velocity mirror, the real pump
  output), lamp outputs (scan pattern + alarm lamps), and the KWS CY300
  power-up calibration / power-off relay sequences (folded in from a
  short-lived separate FC156), all timing via one call to FB555
  "IEC TIMERS" (see below). Originated as a STEP7 export of the live
  PLC's actual block — logic was identical to the now-deleted
  FC104_GLUE_SCALE.awl v0.13 at that point; FC104 was compiled, deployed,
  then renumbered FC104->FC155 and renamed "GLUE_SCALE" -> "GLUE SCALE
  LOGIC" on the real PLC project. Repeatedly reconciled against Michou's
  own parallel online rebuilds, most recently his "Refactor glue scale
  logic" export (2026-10-06) - see v0.31 changelog for what was adopted
  (8s K1 hold, early Scale_ReCalib_Req clear, RED now blinks, Pump_Test
  switch wiring) vs. a bug caught and fixed (WTST only lighting one of
  three test lamps).
- `FB555_IEC_TIMERS.awl` — version 0.7. Michou's design: one wrapper FB
  holding all 11 timers FC155 needs (underfill settle, lamp-scan
  flasher, the 5 power-sequence timers, the 30s startup-stabilization
  timer, and the 3 leak-detection timers) as multi-instance `"TON"`
  (the imported IEC Timer library FB) children, called once per scan
  from FC155 instead of separate top-level instance DBs. Symbol renamed
  "IEC_TIMERS" -> "IEC TIMERS" by Michou (2026-10-06, spaces not
  underscores, matching every other named block). Fixed one bug while
  integrating: `TON_UNDERFILL`'s `PT` was wired to `#LAMP_IN_TIME`
  (copy-paste leftover) instead of `#UNDERFILL_IN_TIME`.
- `DB555_IEC_TIMERS_DB.awl` — version 0.3. `"IEC TIMERS DB"`, the single
  instance DB for FB555, called once from FC155's "Timers" network.
  Renamed to match FB555's symbol, then renumbered DB106 -> DB555 to
  match FB555's own number too (Michou, 2026-10-06: "FB555 Instance is
  DB555... !!!"). See "Power-up calibration workaround" above and
  "S7-300 STL hard limits" at top for why this replaced both the
  classical S5 timers (T50-T56) and the earlier 7-top-level-SFB4-
  instance-DB design.
- `DB4_ABC3000A_DB.awl` — 20-byte raw telegram buffer, filled by SFC14.
- `DB105_GLUE_SCALE_CONTROL_DB.awl` — parsed weight, setpoints, alarm
  limits, scale-fault sentinel, platform-weight sanity limit, pump/alarm
  output bits, alarm reset, lamp scan state, power-on/power-off
  sequencing state, (v0.18) the power-up stabilization/calibration-
  interlock bits, and (v0.20) the leak-detection fields. Version 0.20.
  Symbol is `"GLUE SCALE CONTROL DB"` (spaces) to match what FC155
  actually references.
- `DB105_Online_1.xps` — STEP7 online DB105 snapshot (2026-09-17) used to
  sync the offline source after live-side field edits.
- `Anybus Communicator configuration *.conf` — exported gateway config;
  confirms the "GROSS FILTER" transaction/telegram layout is unchanged.
- `HMI DB 10` — the HMI comms DB (DB10), version 0.3, owned by FC160,
  not this project's source of truth. Referenced here because FC155's
  HMI bridge (above) reads/writes several of its fields by symbol name.
- `DB60_OUTPUTS_DB.awl` — version 0.5. `"OUTPUTS DB"`, the real shared
  output DB (DB60) — GRN_LED_Pnl/RED_LED_Pnl/WHT_SWL_1_Pnl_Pump_Test/
  WHT_SWL_2_Pnl_Calib/WHT_SWL_3_Pnl_Alrm_Ack (now driven by FC158, see
  below), the KILOTECK_Calib_K1/KILOTECK_Power_K2 relay bits, `Pump_Out`
  (was `Output_060`, briefly `Pump_Out_Valve`, not driven by any logic
  here yet), and every other output used by other 410 subsystems (tower
  lamps, Seam_Ctrl_*, etc.), tracked verbatim. Not this subsystem's own
  DB — shared, so most of its fields aren't glue-scale-related. Michou's
  own 2026-10-06 re-export trimmed the speculative placeholder tail and
  renamed several fields with real detail - see its own changelog.
- `DB61_INPUTS_DB.awl` — version 0.2. `"INPUTS DB"` (DB61), the shared
  input DB, tracked verbatim. Not touched by the glue-scale/KWS CY300
  work until v0.2 (2026-10-06), which renamed the 3 panel switches to
  their real function (`WHT_SW_1_Pump_Test`/`WHT_SW_2_ReCalib`/`WHT_SW_
  3_Alarm_Reset`) - now wired into FC155, see "Power-up calibration
  workaround" above.
- `FC158 SCALE PANEL LAMPS` — version 0.4. Standalone, reusable
  panel-lamp driver (white-lamp scan fwd/rev/independent/panic, GRN/RED
  solid/blink) for the 5 lamp bits in DB60. Built as FC157 "PANEL LED
  DRIVE"; renumbered/renamed by Michou (2026-10-06) because FC157 wasn't
  actually free on the live project. GRN/RED blink reads `"M 6.5"`
  directly as of v0.4, not FB556's own ALARM clock. See "Panel LED
  drive" above.
- `DB107_PANEL_LED_CMD_DB.awl` — version 0.2. `"PANEL LED CMD DB"`
  (DB107), FC158's command/status interface — named command bits, not a
  packed mode byte. See "Panel LED drive" above.
- `FB556_PANEL_LED_TIMERS.awl` — version 0.4. `"PANEL LED TIMERS"`
  (renamed from "PANEL_LED_TIMERS" by Michou, spaces not underscores).
  Same multi-instance `"TON"` wrapper pattern as FB555 — SCAN/PANIC
  clocks for FC158, called once per scan. `TON_ALARM`/`ALARM_OUT` are
  dead as of v0.4 (left declared, unused) - see "Panel LED drive" above.
- `DB556_PANEL_LED_TIMERS_DB.awl` — version 0.3. `"PANEL LED TIMERS DB"`,
  the single instance DB for FB556, called once from FC158's "Timers"
  network. Renamed to match FB556's symbol, then renumbered DB108 ->
  DB556 to match FB556's own number too, same reasoning as DB555 above.
  once from FC158's "Timers" network.
- `OB100_COMPLETE_RESTART.awl` — version 0.2. Forces this project's own
  state (ReCalib_Step, latched alarms, HMI trigger pulses, the live
  weight reading, Panel LED command bits, and - v0.2 - the power-up
  stabilization/calibration-interlock bits) back to a safe idle default
  on every PLC restart - DB "actual values" otherwise survive a power
  cycle battery/cap-backed, so without this a reboot mid-recalibration
  would resume as if nothing happened while the relays are actually
  de-energized. Does **not** touch operator setpoints. **Confirmed
  (2026-10-06): OB100 already exists on the live PLC** - this file's
  networks need to be pasted into the end of that existing block, never
  downloaded as a replacement.

## Still open

- **Leak detection (v0.36) thresholds/durations are all placeholders** -
  `FillStallMinRise_SP` (1kg), the 60s stall timeout, the 5-minute fill
  timeout, `MaxDrainDrop_SP` (5kg), and the 30s idle-check window. None
  of these are based on real process data - confirm/tune against the
  actual fill rate and normal glue-consumption rate. See "Leak
  detection" above.
- **"Interrupt machine feed" — confirmed deferred (Michou, 2026-10-06):**
  for now, `Alarm_PossibleLeak` only blocks the fill pump and lights the
  RED lamp, same as the other three alarms. No cross-subsystem
  interlock to the downstream (Rotaliner/Tuber) process is wired - that
  stays a manual operator response until a specific output/mechanism is
  identified and asked for.
- **Confirm whether OB100 already exists on the live PLC project.** This
  repo can't see the whole project - if it does, `OB100_COMPLETE_RESTART.
  awl`'s 3 networks need to be pasted into the end of that existing
  block, not downloaded as a replacement (would silently delete whatever
  else it resets for other 410 subsystems). If it doesn't exist yet,
  this file can be used as-is.
- `Reset_Alarms` is now wired to `WHT_SW_3_Alarm_Reset` (v0.31, my own
  inference, not confirmed by Michou — see above).
- FC158's white-lamp scan (now driven by FB556's clock, superseding
  FB555's `LAMP_IN`/`LAMP_OUT`/`LampScanClockMem`/`LampScanStep`) is
  hand-traced but not bench-verified — confirm the 500ms scan period, the
  150ms Panic strobe, and the 500ms GRN/RED blink on real hardware.
- No HMI control wired to FC158's `WHT_Panic`/`GRN_Blink` yet — Michou
  asked for the capability, not a specific trigger. `RED_Blink` is no
  longer free for independent test use - FC155 v0.31 drives it directly
  for the alarm lamp (see "Control logic" above).
- Confirm FB555/DB555 and the new FB556/DB107/DB556/FC158 are genuinely
  free block numbers on the live PLC — this repo can't see the whole
  project, same unresolved caution the classical timer numbers (T50-T56,
  now retired) never got fully closed out on either. FC157->FC158 (see
  "Panel LED drive" above) already proved this caution was warranted.
- `Hysteresis`, `SpareReal2`, `Spare_INT`, `Scale_Powered_On`,
  `Actual_Transfered_Weight` (the DB105 copy) and `Spare_11..Spare_15`
  exist in DB105 but aren't wired to anything in FC155 yet — no
  confirmed intended behavior for any of them.
- The touch panel's own screen project needs its tags re-pointed to
  match `"HMI DB"` v0.3's restructured offsets — outside this repo, can't
  be done from here.
- None of the three power-sequence triggers (`Scale_ReCalib_Req`,
  `Scale_PowerOFF_Pulse`, `Scale_PowerON_Pulse`) are wired to HMI
  buttons yet.
- `PlatformMinWeight_SP` (30kg) is a best-guess default from Michou's
  "Platform free weight" description — confirm against the actual empty
  platform reading before relying on it to gate the pump.
- Many NETWORK titles and inline comments throughout FC155/DB105 are
  long enough to trigger STEP7's "Comment or title length too big"
  warning (non-fatal, just silently truncated) — not fixed yet, lower
  priority than the fatal errors this round addressed.
