# Glue Scale Weight Monitor / Valve Control — 410 Line

Reads a KILO TECK KWS CY300 glue-transfer scale via serial ASCII, brings it
into the S7-315-2 DP PLC through an Anybus Communicator (PROFIBUS gateway),
parses the ASCII weight into a REAL, and drives a fill valve with hysteresis
plus latching over/underfill alarms.

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
| 8 | Decimal digit |
| 9-10 | `k`,`g` |
| 11-19 | Padding (0x00 / spaces) |

FC104 parses Byte_0-6 and Byte_8 into `GLUE_SCALE_CONTROL_DB.GrossWeight_Actual : REAL`.
Byte_7/9/10 are checked against their fixed values each scan; any mismatch
(or a non-zero SFC14 RET_VAL) sets `ParseError` and holds the last-good
weight/valve/alarm state rather than acting on a bad telegram.

## Control logic (GLUE_SCALE_CONTROL_DB / DB105)

- **Valve_Open**: two explicit thresholds, no separate hysteresis
  subtraction. Opens when `GrossWeight_Actual < Fill_Start_SP`; closes at
  `GrossWeight_Actual >= Fill_Stop_SP`. (`Hysteresis` is still declared in
  DB105 for struct-layout compatibility but is no longer read here.)
- **Alarm_Overfill**: latching, sets if `GrossWeight_Actual > AlarmLimit_Overfill`.
- **Alarm_Underfill**: 5 seconds after Valve_Open falls (timer T50), checks
  `GrossWeight_Actual < AlarmLimit_Underfill` once at that instant; latching.
- **Alarm_ScaleFault**: latching, sets if `GrossWeight_Actual` matches
  `NegativeUnderScore_Value` — the KWS CY300's fixed sentinel telegram
  (confirmed live: 5222222, all 7 digits of the integer-part field, no
  leading spaces) sent in place of a real reading when its measurement
  goes negative. Also gates valve/alarm updates the same way ParseError
  does, so the valve doesn't react to the sentinel as if it were a weight.
- **Reset_Alarms**: HMI-driven input bit. While TRUE, clears
  `Alarm_Overfill`, `Alarm_Underfill` and `Alarm_ScaleFault` together
  every scan (level-conditioned, not edge — safe as a momentary
  acknowledge button, since any alarm whose fault condition is still
  present just re-latches the next scan). Runs unconditionally, even on
  a scan where ParseError/Alarm_ScaleFault would otherwise skip the rest
  of the block, so a latched fault can always be cleared. The PLC side
  is wired; connecting an actual HMI button to this bit is still open.

## Files

- `FC104_GLUE_SCALE.awl` — SFC14 reads, ASCII parse, valve control,
  alarm latching and reset. Version 0.13.
- `DB4_ABC3000A_DB.awl` — 20-byte raw telegram buffer, filled by SFC14.
- `DB105_GLUE_SCALE_CONTROL_DB.awl` — parsed weight, setpoints, alarm
  limits, scale-fault sentinel, valve/alarm output bits, alarm reset.
  Version 0.4.
- `DB105_Online_1.xps` — STEP7 online DB105 snapshot (2026-09-17) used to
  sync the offline source after live-side field edits.
- `Anybus Communicator configuration *.conf` — exported gateway config;
  confirms the "GROSS FILTER" transaction/telegram layout is unchanged.

## Still open

- HMI button/screen wiring to pulse `Reset_Alarms` — the PLC-side reset
  logic exists, nothing drives the bit yet.
- Confirm T50 isn't used elsewhere in the 410 project.
- `Hysteresis`, `SpareReal`, and `Scale_Powered_On` exist in DB105 but
  aren't wired to anything in FC104 yet — no confirmed intended behavior
  for any of them.
