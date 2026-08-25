# Implementation Checklist - Quick Reference

**Before implementing a feature, work through this in order:**

## Phase 1: Explore
- [ ] Read the relevant issue/task fully
- [ ] Read the relevant sections of `SPEC.md`
- [ ] Read the relevant sections of `ARCHITECTURE.md`
- [ ] Explore existing code before writing anything — check
      `BoatHacks/halser-hwt3100-interface` (the parent project this was
      forked from) for reusable patterns (`ExpiringValue`,
      `ConfigItem`/`PersistingObservableValue`,
      `TaskQueueProducer`-based I/O tasks) before writing something new;
      most of the N2K/SignalK plumbing is unchanged from it

## Phase 2: Plan
- [ ] Think through the approach and alternatives
- [ ] Write a short implementation plan in `docs/plans/` (see
      `docs/plans/TEMPLATE.md`)
- [ ] Identify test scenarios up front, including hardware-in-the-loop
      cases — this project's protocol layer was written against
      WitMotion's published register protocol without a physical WT901B
      to test against (SPEC.md §11); anything touching the sensor
      protocol needs real-hardware verification eventually, not just
      unit tests against the documented format

## Phase 3: Test
- [ ] Where logic can be isolated from hardware (the binary frame
      parser, the register-write command formatters, the calibration
      offset math), write it so it's unit tested via `pio test -e
      native`
- [ ] For anything that can't be meaningfully unit tested (serial I/O
      timing, actual N2K bus behavior), plan the manual verification step
      instead — write down what "working" looks like before implementing

## Phase 4: Implement
- [ ] Write code to satisfy the plan
- [ ] Build frequently (`pio run`) while working, not just at the end
- [ ] **Never add a code path that writes arbitrary/unvalidated register
      addresses or data bytes to the WT901B serial link.** Any new write
      capability must go through `HWT901BSerialIO::SendCommand()`'s
      closed `HWT901BCommand` enum, or a new bounded/clamped formatter
      following the pattern in `hwt901b_bandwidth_command.h`/
      `hwt901b_rate_command.h`/`hwt901b_baud_command.h`
      (ARCHITECTURE.md §2.1, §6). If a change would add or touch a
      factory-reset SAVE write or any CALSW value beyond `0`/`7`, stop
      and treat it as reopening a closed safety decision (SPEC.md §1.2,
      §9.3), not a routine addition.

## Phase 5: Verify
- [ ] Check edge cases, not just the happy path — the WT901B losing
      power/wiring, corrupted/misaligned frames, WiFi/SignalK server
      unreachable, N2K bus not present
- [ ] Confirm the change matches `SPEC.md`
- [ ] Confirm the change follows `ARCHITECTURE.md`
- [ ] If hardware is available: verify against a real WT901B module, not
      just against the documented register protocol — SPEC.md §11 lists
      everything this project could not independently confirm without
      one

## Phase 6: Document & Commit
- [ ] Update SPEC.md/ARCHITECTURE.md if this change altered what they
      describe
- [ ] Remove any temporal language from comments ("new", "recently
      added") — comments should read correctly a year from now
- [ ] Firmware builds cleanly (`pio run`)
- [ ] Commit with a message that explains *why*, referencing the issue

---

## Common Mistakes to Avoid

**Don't:**
- Jump straight to coding before reading SPEC.md/ARCHITECTURE.md
- Assume the parent HWT3100 project's hardware-capability claims still
  apply here without checking — the WT901B *does* have pitch/roll and a
  real gyroscope; several of the parent project's "permanent exclusions"
  (§9.3 there) are implemented features here (SPEC.md §1.2, §5.1)
- Loosen the `HWT901BCommand`/bandwidth/rate/baud write paths to accept
  arbitrary register addresses or values "for flexibility" — that's
  exactly the design the closed enums/clamped formatters exist to
  prevent (SPEC.md §10, ARCHITECTURE.md §6)
- Treat SPEC.md §11's open questions as settled — they're genuine gaps
  from not having physical hardware in this environment, not hedging
  language to skim past
- Leave SPEC.md/ARCHITECTURE.md stale after a change that contradicts
  them

**Do:**
- Explore before planning, plan before coding
- Write down the plan somewhere reviewable, even briefly
- Verify against the docs, not just against your own memory of the task
- When in doubt about the WT901B's actual behavior, prefer real hardware
  testing over the documented register protocol — the parent project was
  burned once by exactly this kind of manual-derived assumption (its own
  CHANGELOG records it)
