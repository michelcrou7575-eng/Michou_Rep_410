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
- **Alarm_Underfill**: 5 seconds after Pump_ON falls (timer T50), checks
  `GrossWeight_Actual < AlarmLimit_Underfill_SP` once at that instant; latching.
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
  `Alarm_Underfill` is latched, `Pump_ON` is forced off and held off (blocks
  automatic re-engagement) until `Reset_Alarms` clears the alarm.
- **Reset_Alarms**: HMI-driven input bit. While TRUE, clears
  `Alarm_Overfill`, `Alarm_Underfill` and `Alarm_ScaleFault` together
  every scan (level-conditioned, not edge — safe as a momentary
  acknowledge button, since any alarm whose fault condition is still
  present just re-latches the next scan). Runs unconditionally, even on
  a scan where ParseError/Alarm_ScaleFault would otherwise skip the rest
  of the block, so a latched fault can always be cleared. The PLC side
  is wired; connecting an actual HMI button to this bit is still open.

## Transfered weight

`"HMI DB".Actual_Transfered_Weight` = `GrossWeight_Actual` −
`GrossWeight_Actual_Mem`, recomputed every scan. `GrossWeight_Actual_Mem` is
a snapshot of the weight taken once, on the scan `Pump_ON` is newly
commanded on — i.e. this tracks how much has been transferred since the
pump last started, confirmed against the real pump-start event, not a
free-running or reset-driven total.

## Manual test and indicator lamps (all in "OUTPUTS DB" / DB60, shared
## with other subsystems — see `DB60_OUTPUTS_DB.awl`)

- **Glue_Fill_Pump_Test** (`"HMI DB"`, manual HMI bit): overrides
  `Glue_Fill_Pump_ON` on regardless of `Pump_ON`'s automatic state — and
  forces `WHT_SWL_1`/`WHT_SWL_2`/`WHT_SWL_3` all on together, in place of
  the scan pattern below. Since Test only ever forces the output on (never
  off), this is implemented as a plain OR against the automatic decision,
  not a conditional branch — same result, less code. Runs unconditionally
  (not gated by ParseError/Alarm_ScaleFault), so manual test still works
  during a scale fault.
- **WHT_SWL_1/2/3 scan pattern**: while `Pump_ON` is true (and Test is not
  active), these three lamps cycle 1→2→3→1… as a running-light "pump active"
  indicator. Driven by a 500ms one-scan-pulse flasher (`LampScanClockMem`,
  timer T51 — confirm unused elsewhere in the 410 project, same caution
  already flagged for T50) advancing `LampScanStep` (1/2/3, resets to 1 when
  the pump stops). **Not bench-verified** — the flasher is a standard S7 STL
  idiom, traced through by hand, but hasn't been confirmed on real hardware.
- **RED_LED**: on if `Alarm_Overfill` OR `Alarm_Underfill` OR
  `Alarm_ScaleFault` is latched. **GRN_LED**: on if none are. (Michou's spec
  named only Overfill/Underfill for RED and "no alarm" for GRN —
  `Alarm_ScaleFault` was folded into both here so GRN's "no alarm" claim
  stays accurate; flag if ScaleFault shouldn't be included.)

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

K2 is a momentary button-press simulation, not a plain on/off: per
Michou (2026-10-02), a **short 0.5s pulse powers the scale ON**, a
**long 3s pulse powers it OFF**.

FC155 runs three independent sequences, with sequencing state in DB105
("GLUE SCALE CONTROL DB" — this subsystem's own state, not the shared
output DB):

- **Power-on + calibrate** (`Scale_ReCalib_Req` rising edge, ignored
  while already running): recalibrating needs a full power cycle, not
  just toggling K1 while the scale is already running — so this first
  pulses K2 for 3s to power the scale OFF (`ReCalib_Step` 1, reusing
  the same T54 timer as the standalone power-off pulse below), then
  energizes K1, holds it on for a flat 5s (`ReCalib_Step` 2→5), pulsing
  K2 on for 0.5s inside that same window to power the scale back up
  while the calibration shunt is already connected — the scale sees the
  known shunt reference the moment it boots. K1 releases when the 5s
  elapses. The only one of the three with multi-step state
  (`ReCalib_Step`); edge-detected via raw symbol `"FP 201.1"`.
- **Manual power-off pulse** (`Scale_PowerOFF_Pulse`, ignored while
  `ReCalib_Step` is mid-sequence): a single 3s K2 pulse (T56), no K1
  involved. No step counter — gated directly on the bit's own level
  (safe without separate edge memory, since the timer only truly
  restarts on a rising edge); auto-clears the bit when the pulse
  completes.
- **Manual power-on pulse** (`Scale_PowerON_Pulse`, same guard/idiom):
  a single 500ms K2 pulse (T55), auto-clears when done.

Each sequence guards against starting while another is mid-run, so
K1/K2 are never driven by more than one action in the same scan.
Timers: T52 (0.5s power-on pulse inside ReCalib), T53 (5s K1 hold), T54
(3s power-off pulse inside ReCalib), T55 (standalone 500ms power-on
pulse), T56 (standalone 3s power-off pulse) — confirmed unused
elsewhere in the 410 project, same check as T50/T51.

**Still open / flagged for confirmation**:
- None of `Scale_ReCalib_Req`/`Scale_PowerOFF_Pulse`/`Scale_PowerON_Pulse`
  are wired to HMI buttons yet.
- `_Pnl` suffix: confirmed real for all five lamp outputs now
  (`RED_LED_Pnl`/`GRN_LED_Pnl` by a real compile error; `WHT_SWL_1/2/3_Pnl`
  by Michou's next working online version using them) — FC155 v0.25,
  DB60 v0.3.
- **"ReCalib_Step stuck at 3" (2026-10-02)**: fixed in FC155 v0.25 by
  adding `SET;` before the `S`/`SD`/`R` coil at the start of
  `RCS1`/`RCS2`/`RCS3`/`RCS5` (each reached via a jump, which can't be
  trusted to leave RLO=1 — see "S7-300 STL hard limits" above). Not
  100% certain this was the actual cause — if steps still stick after
  this fix, the next suspect is **Timer 52 colliding with something
  elsewhere in the live project** this repo can't see (the T50/T51
  collision caution never got fully closed out, and the same applies to
  T52-T56).
- The "Update" network (`#UPDATE` := rising edge of `"M 6.7"`) now gates
  "Detect scale fault" and "Overfill alarm" (and "Platform weight sanity
  check" together with `Pump_ON`) behind `Pump_ON OR #UPDATE` instead of
  running every scan — adopted from Michou's own online edit, but
  **not independently confirmed as intentional**: if `M 6.7` only
  pulses rarely, this means the scale-fault and overfill checks only
  evaluate occasionally rather than continuously. Worth confirming given
  these are safety-relevant checks.

## Files

- `FC155 GLUE SCALE LOGIC` — canonical source, version 0.25. SFC14 reads,
  ASCII parse (INT weight, decimal digit discarded), pump control with
  overfill/underfill interlock, platform-weight sanity check + scale-fault
  pump interlock, alarm latching and reset, HMI DB bridge (setpoints, live
  weight, transfered-weight calc, bottomer-velocity mirror, the real pump
  output), lamp outputs (scan pattern + alarm lamps), and the KWS CY300
  power-up calibration / power-off relay sequences (folded in from a
  short-lived separate FC156). Originated as a STEP7 export of the live
  PLC's actual block — logic was identical to the now-deleted
  FC104_GLUE_SCALE.awl v0.13 at that point; FC104 was compiled, deployed,
  then renumbered FC104->FC155 and renamed "GLUE_SCALE" -> "GLUE SCALE
  LOGIC" on the real PLC project. Repeatedly reconciled against Michou's
  own parallel online rebuilds (v0.18/v0.19, v0.21).
- `DB4_ABC3000A_DB.awl` — 20-byte raw telegram buffer, filled by SFC14.
- `DB105_GLUE_SCALE_CONTROL_DB.awl` — parsed weight, setpoints, alarm
  limits, scale-fault sentinel, platform-weight sanity limit, pump/alarm
  output bits, alarm reset, lamp scan state, power-on/power-off
  sequencing state. Version 0.15. Symbol is `"GLUE SCALE CONTROL DB"`
  (spaces) to match what FC155 actually references.
- `DB105_Online_1.xps` — STEP7 online DB105 snapshot (2026-09-17) used to
  sync the offline source after live-side field edits.
- `Anybus Communicator configuration *.conf` — exported gateway config;
  confirms the "GROSS FILTER" transaction/telegram layout is unchanged.
- `HMI DB 10` — the HMI comms DB (DB10), version 0.3, owned by FC160,
  not this project's source of truth. Referenced here because FC155's
  HMI bridge (above) reads/writes several of its fields by symbol name.
- `DB60_OUTPUTS_DB.awl` — version 0.3. `"OUTPUTS DB"`, the real shared
  output DB (DB60) — GRN_LED/RED_LED/WHT_SWL_1/2/3 (already referenced
  by FC155), the new KILOTECK_Calib_K1/KILOTECK_Power_K2 relay bits, and
  every other output used by other 410 subsystems (tower lamps,
  Seam_Ctrl_*, etc.), tracked verbatim. Not this subsystem's own DB —
  shared, so most of its fields aren't glue-scale-related.
- `DB61_INPUTS_DB.awl` — version 0.1. `"INPUTS DB"` (DB61), the shared
  input DB, tracked verbatim. Not touched by the glue-scale/KWS CY300
  work — included for completeness since it was shared alongside DB60.

## Still open

- HMI button/screen wiring to pulse `Reset_Alarms` — the PLC-side reset
  logic exists, nothing drives the bit yet.
- The lamp-scan flasher (`LampScanClockMem`/`LampScanStep`, T51) is
  hand-traced but not bench-verified — confirm the 500ms period and
  on/off behavior on real hardware.
- Confirm T50 and T51 aren't used elsewhere in the 410 project.
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
