# Glue Scale Weight Monitor / Valve Control — 410 Line

Reads a KILO TECK KWS CY300 glue-transfer scale via serial ASCII, brings it
into the S7-315-2 DP PLC through an Anybus Communicator (PROFIBUS gateway),
parses the ASCII weight into an INT (whole kg — the scale's decimal digit is
discarded, fractional precision isn't needed), and drives a fill valve with
hysteresis plus latching over/underfill alarms.

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
transaction doesn't need sub-kg precision, and every live setpoint was
already a whole number. Byte_7/9/10 are checked against their fixed values
each scan; any mismatch (or a non-zero SFC14 RET_VAL) sets `ParseError` and
holds the last-good weight/valve/alarm state rather than acting on a bad
telegram.

## Control logic ("GLUE SCALE CONTROL DB" / DB105)

- **Valve_Open**: two explicit thresholds, no separate hysteresis
  subtraction. Opens when `GrossWeight_Actual < Fill_Start_SP`; closes at
  `GrossWeight_Actual >= Fill_Stop_SP`. (`Hysteresis` is still declared in
  DB105 for struct-layout compatibility but is no longer read here.)
  Mirrored onto the real actuator bit, `"HMI DB".Glue_Fill_Pump_ON` — see
  HMI bridge section below; before this, Valve_Open drove no physical
  output at all.
- **Alarm_Overfill**: latching, sets if `GrossWeight_Actual > AlarmLimit_Overfill`.
- **Alarm_Underfill**: 5 seconds after Valve_Open falls (timer T50), checks
  `GrossWeight_Actual < AlarmLimit_Underfill` once at that instant; latching.
- **Alarm_ScaleFault**: latching, sets if the raw parsed integer matches
  `NegativeUnderScore_Value` (DINT, 5222222) — the KWS CY300's fixed
  sentinel telegram (all 7 digits of the integer-part field, no leading
  spaces) sent in place of a real reading when its measurement goes
  negative. The comparison runs on the wide parse temp *before* it's
  truncated into the INT `GrossWeight_Actual` — 5222222 doesn't fit in
  16-bit INT range, so comparing the already-truncated value would never
  match. Also gates valve/alarm updates the same way ParseError does, so
  the valve doesn't react to the sentinel as if it were a weight.
- **Reset_Alarms**: HMI-driven input bit. While TRUE, clears
  `Alarm_Overfill`, `Alarm_Underfill` and `Alarm_ScaleFault` together
  every scan (level-conditioned, not edge — safe as a momentary
  acknowledge button, since any alarm whose fault condition is still
  present just re-latches the next scan). Runs unconditionally, even on
  a scan where ParseError/Alarm_ScaleFault would otherwise skip the rest
  of the block, so a latched fault can always be cleared. The PLC side
  is wired; connecting an actual HMI button to this bit is still open.

## HMI bridge ("HMI DB" / DB10)

FC155 mirrors six fields to/from `"HMI DB"` (DB10) every scan:

- `"HMI DB".Fill_Start_Weight_SP` → `Fill_Start_SP`,
  `"HMI DB".Fill_Stop_Weight_SP` → `Fill_Stop_SP`,
  `"HMI DB".Overfill_Weight_SP` → `AlarmLimit_Overfill`, and
  `"HMI DB".Underfill_Weight_SP` → `AlarmLimit_Underfill` — all four
  operator-entered setpoints flow HMI → scale, one-way, unconditionally
  (not gated by ParseError/Alarm_ScaleFault — the touch panel's own
  numeric-entry widget is the display of record for what was last
  typed, so nothing needs to be mirrored back for these).
- `GrossWeight_Actual` → `"HMI DB".Actual_Glue_Weight` — the live reading
  flows scale → HMI, for display, unconditionally.
- `Valve_Open` → `"HMI DB".Glue_Fill_Pump_ON` — the actual pump output.
  Runs inside the same ParseError/Alarm_ScaleFault-gated region as
  `Valve_Open` itself, so it only updates on scans where the valve
  logic itself ran; during a fault it simply isn't re-evaluated, same
  "hold last state" behavior as the rest of the gated block. Found by
  checking what, if anything, drove a real output from `Valve_Open` -
  nothing did, anywhere in this project, until this was wired.
  `"HMI DB".Glue_Fill_Pump_Test` is a separate manual bit that still
  isn't wired to anything - see "Still open".

The `Overfill_Weight_SP`/`Underfill_Weight_SP` pair didn't originally
exist in DB10 — added (HMI DB v0.2) by repurposing two of its three
`Spare_INT` slots, after the physical "Glue InFeed Manager" touch panel
turned out to already have "Over Filled Weight"/"Under Filled Weight"
setpoint fields on screen with nothing backing them.

`"HMI DB"` and DB105 both happen to have 5 consecutive INT fields at
offsets 0/2/4/6/8, but `"HMI DB".Actual_Transfered_Weight`/
`Bottomer_Velocity` (6.0/8.0) and DB105's `AlarmLimit_Overfill`/
`AlarmLimit_Underfill` (also 6.0/8.0) are unrelated fields that only
share a byte offset by coincidence — `Overfill_Weight_SP`/
`Underfill_Weight_SP` (the fields that actually correspond to
`AlarmLimit_Overfill`/`AlarmLimit_Underfill`) live at 14.0/16.0 in
DB10, not 6.0/8.0. The sync is wired by symbol name, one field at a
time — never as a block copy across a matching address range.

## Files

- `FC155 GLUE SCALE LOGIC` — canonical source, version 0.17. SFC14 reads,
  ASCII parse (INT weight, decimal digit discarded), valve control, alarm
  latching and reset, HMI DB bridge (setpoints, live weight, and the
  real pump output). Originated as a STEP7 export of the
  live PLC's actual block — logic was identical to the now-deleted
  FC104_GLUE_SCALE.awl v0.13 at that point; FC104 was compiled, deployed,
  then renumbered FC104->FC155 and renamed "GLUE_SCALE" ->
  "GLUE SCALE LOGIC" on the real PLC project.
- `DB4_ABC3000A_DB.awl` — 20-byte raw telegram buffer, filled by SFC14.
- `DB105_GLUE_SCALE_CONTROL_DB.awl` — parsed weight, setpoints, alarm
  limits, scale-fault sentinel, valve/alarm output bits, alarm reset.
  Version 0.7. Symbol is `"GLUE SCALE CONTROL DB"` (spaces) to match what
  FC155 actually references.
- `DB105_Online_1.xps` — STEP7 online DB105 snapshot (2026-09-17) used to
  sync the offline source after live-side field edits.
- `Anybus Communicator configuration *.conf` — exported gateway config;
  confirms the "GROSS FILTER" transaction/telegram layout is unchanged.
- `HMI DB 10` — the HMI comms DB (DB10), version 0.2, owned by FC160,
  not this project's source of truth. Referenced here because FC155's
  HMI bridge (above) reads/writes six of its fields by symbol name.

## Still open

- HMI button/screen wiring to pulse `Reset_Alarms` — the PLC-side reset
  logic exists, nothing drives the bit yet.
- `"HMI DB".Actual_Transfered_Weight` has no source anywhere in this
  project. No specification exists for what it should count (cumulative
  since reset? per fill-cycle? fed from something other than this
  scale entirely?) - needs that answered before anything writes to it.
- `"HMI DB".Glue_Fill_Pump_Test` — exists in DB10, unused. How a manual
  test request should interact with `Valve_Open`'s automatic decision
  (override it, gate it, run independently) isn't specified.
- Confirm T50 isn't used elsewhere in the 410 project.
- `Hysteresis`, `SpareReal`/`SpareReal1`/`SpareReal2`/`SpareReal3`, and
  `Scale_Powered_On` exist in DB105 but aren't wired to anything in FC155
  yet — no confirmed intended behavior for any of them.
- The touch panel's own screen project needs `Overfill_Weight_SP`/
  `Underfill_Weight_SP`'s tags pointed at DB10's new offsets (14.0/16.0)
  — outside this repo, can't be done from here.
