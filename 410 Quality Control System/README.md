# Glue Scale Weight Monitor / Pump Control — 410 Line

Reads a KILO TECK KWS CY300 glue-transfer scale via serial ASCII, brings it
into the S7-315-2 DP PLC through an Anybus Communicator (PROFIBUS gateway),
parses the ASCII weight into an INT (whole kg — the scale's decimal digit is
discarded, fractional precision isn't needed), and drives a glue fill pump
with a start/stop weight band plus latching over/underfill alarms.

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
- **Alarm_ScaleFault**: latching, sets if the raw parsed integer matches
  `NegativeUnderScore_Value` (DINT, 5222222) — the KWS CY300's fixed
  sentinel telegram (all 7 digits of the integer-part field, no leading
  spaces) sent in place of a real reading when its measurement goes
  negative. The comparison runs on the wide parse temp *before* it's
  truncated into the INT `GrossWeight_Actual` — 5222222 doesn't fit in
  16-bit INT range, so comparing the already-truncated value would never
  match. Also gates pump/alarm updates the same way ParseError does, so
  the pump doesn't react to the sentinel as if it were a weight.
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

## Manual test and indicator lamps (all in "OUTPUTS DB", shared with other
## subsystems — not tracked in this repo)

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

## Files

- `FC155 GLUE SCALE LOGIC` — canonical source, version 0.19. SFC14 reads,
  ASCII parse (INT weight, decimal digit discarded), pump control with
  overfill/underfill interlock, alarm latching and reset, HMI DB bridge
  (setpoints, live weight, transfered-weight calc, bottomer-velocity
  mirror, the real pump output), and lamp outputs (scan pattern + alarm
  lamps). Originated as a STEP7 export of the live PLC's actual block —
  logic was identical to the now-deleted FC104_GLUE_SCALE.awl v0.13 at
  that point; FC104 was compiled, deployed, then renumbered FC104->FC155
  and renamed "GLUE_SCALE" -> "GLUE SCALE LOGIC" on the real PLC project.
  Later independently rebuilt online past v0.15 by Michou (Pump_ON
  rename, transfered-weight tracking, HMI restructure) in parallel with
  this repo's own v0.16/v0.17 — both reconciled together at v0.18/v0.19.
- `DB4_ABC3000A_DB.awl` — 20-byte raw telegram buffer, filled by SFC14.
- `DB105_GLUE_SCALE_CONTROL_DB.awl` — parsed weight, setpoints, alarm
  limits, scale-fault sentinel, pump/alarm output bits, alarm reset, lamp
  scan state. Version 0.9. Symbol is `"GLUE SCALE CONTROL DB"` (spaces)
  to match what FC155 actually references.
- `DB105_Online_1.xps` — STEP7 online DB105 snapshot (2026-09-17) used to
  sync the offline source after live-side field edits.
- `Anybus Communicator configuration *.conf` — exported gateway config;
  confirms the "GROSS FILTER" transaction/telegram layout is unchanged.
- `HMI DB 10` — the HMI comms DB (DB10), version 0.3, owned by FC160,
  not this project's source of truth. Referenced here because FC155's
  HMI bridge (above) reads/writes several of its fields by symbol name.

## Still open

- HMI button/screen wiring to pulse `Reset_Alarms` — the PLC-side reset
  logic exists, nothing drives the bit yet.
- The lamp-scan flasher (`LampScanClockMem`/`LampScanStep`, T51) is
  hand-traced but not bench-verified — confirm the 500ms period and
  on/off behavior on real hardware.
- Confirm T50 and T51 aren't used elsewhere in the 410 project.
- `Hysteresis`, `SpareReal`/`SpareReal1`/`SpareReal2`/`SpareReal3`, and
  `Scale_Powered_On` exist in DB105 but aren't wired to anything in FC155
  yet — no confirmed intended behavior for any of them.
- The touch panel's own screen project needs its tags re-pointed to
  match `"HMI DB"` v0.3's restructured offsets — outside this repo, can't
  be done from here.
