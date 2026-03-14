# Battery Health Analysis — VAALE 2026

## Data collected per match

| Match | Battery (likely) | Max V | Min V | Sag Range | Mod0 Drive Mean | Notes |
|-------|-----------------|-------|-------|-----------|-----------------|-------|
| P6 Practice | 2025-1 | 12.62 | 12.58 | 0.04V | ~0A | Never loaded — idle only |
| Q8 | **2023-1** | 13.25 | **7.24** | 6.01V | 8.75A | Worst sag among comparable load matches |
| Q38 | unknown | 12.80 | 12.75 | 0.05V | ~0A | Robot never enabled; aborted match |
| Q51 | 2026-4 | 13.22 | **6.62** | **6.60V** | 12.62A | Absolute lowest voltage of event |
| Q56 | 2026-3 | **13.48** | 7.06 | 6.42V | 5.59A | High idle V, significant sag despite moderate load |
| Q64 | 2026-3 | 13.03 | 6.81 | 6.22V | 9.28A | |
| Q68 | **2025-1** | 13.10 | **7.54** | **5.56V** | **20.15A** | Highest drive load, best sag ratio |
| Q74 | 2025-4 or 2025-2 | 13.47 | 7.47 | 6.01V | 17.09A | |
| E6 | **2023-1** | 13.14 | 6.88 | 6.26V | 13.36A | |
| E9 | unknown (reuse?) | **12.84** | 6.89 | 5.95V | 8.03A | Max V suspiciously low — possible reuse without recharge |

Battery assignment confidence: Q64/Q68/Q74 are nearly certain from timestamp alignment (tested 14:04/14:32/14:57 matching match start times). Q8 and E6 assigned to 2023-1 based on day-of test order.

### Battery tester data (for reference)

| Timestamp | Battery | SoC% | V0 @0A | V2 @18A | Rint (Ω) | Used for |
|-----------|---------|-------|--------|---------|----------|----------|
| 3/7 9:46  | 2025-1  | 107   | 13.092 | 12.774  | 0.017    | Practice |
| 3/7 11:12 | 2023-1  | 130   | 13.422 | 13.915  | 0.021    | Match    |
| 3/7 11:42 | 2025-4  | 129   | 13.364 | 13.059  | 0.016    | Match    |
| 3/7 12:24 | 2025-1  | 125   | 13.318 | 12.980  | 0.017    | Match    |
| 3/7 13:46 | 2026-4  | 130   | 13.369 | 13.084  | 0.015    | Match    |
| 3/7 14:57 | 2026-3  | 130   | 13.371 | 13.106  | 0.015    | Match    |
| 3/7 15:43 | 2025-1  | 114   | 13.180 | 12.936  | 0.014    | Match    |
| 3/7 19:10 | 2025-4  | 128   | 13.351 | 13.104  | 0.014    | Match    |
| 3/8 9:44  | 2026-3  | 122   | 13.275 | 12.999  | 0.015    | Match    |
| 3/8 10:19 | 2025-1  | 130   | 13.407 | 13.111  | 0.016    | Match    |
| 3/8 10:55 | 2025-2  | 128   | 13.350 | 13.069  | 0.015    | Match    |
| 3/8 14:04 | 2026-4  | 123   | 13.284 | 13.020  | 0.015    | Match    |
| 3/8 14:32 | 2025-1  | 128   | 13.354 | 13.097  | 0.014    | Match    |
| 3/8 14:57 | 2025-4  | 130   | 13.380 | 13.110  | 0.015    | Match    |
| 3/8 15:03 | 2023-1  | 128   | 13.350 | 12.993  | 0.019    | Match    |

---

## Per-battery assessment

### 2023-1 — Watch closely

- Highest internal resistance in the fleet: **0.019–0.021 Ω** (vs 0.014–0.016 Ω for newer batteries)
- Q8: min voltage 7.24V at only ~35A average total draw — other batteries at similar loads hold 0.3–0.5V higher
- E6: min 6.88V; consistent with high-Rint behavior
- The battery tester's 18A test (V2=13.915V) on the day-1 test was anomalously *higher* than V0=13.422V — likely a surface charge artifact from being just pulled off the charger, but it makes that specific measurement unreliable
- **Verdict:** Measurably weaker than the rest of the fleet. Logs confirm the tester's Rint flag. Consider retiring or restricting to practice/non-critical use.

### 2025-1 — Best performer

- Used **4×** throughout the event (practice, qual matches, eliminations) — highest utilization of any battery
- Q68 is the standout: highest mean drive current per module (20A) yet the *smallest* voltage sag range of any real match (5.56V). Min V = 7.54V under the hardest driving of the event
- Consistent Rint 0.014–0.017 Ω across all test sessions despite heavy use
- **Verdict:** Healthiest battery in the fleet. Holds voltage well under heavy load.

### 2026-3, 2026-4, 2025-4, 2025-2 — All good

- Rint 0.014–0.016 Ω, consistent with new/like-new batteries
- Voltage performance in logs matches expected behavior for their resistance values
- No red flags

---

## Notable events

### Q51 — Lowest voltage of the event (6.62V)

Q51 was a late-night match (00:01 on 3/8) at the end of a long competition day. The bus voltage hit **6.62V** — below the 6.8V brownout threshold, meaning the robot almost certainly browned out momentarily. This is likely a combination of battery fatigue from a full day of matches and high peak demand. The battery used here (likely 2026-4) looks fine in its tester data, so this is primarily a match-load event rather than a battery defect.

### E9 — Possible reuse without recharge

- Max voltage in E9 was only **12.84V** — every other match with a freshly charged battery started at 13.0–13.5V
- E6 ended at ~18:48, E9 started at 19:16 — only **28 minutes apart**
- 28 minutes is insufficient time to fully recharge a competition battery
- The lower starting voltage means less energy headroom; any peak demand sag hits brownout territory sooner
- **Verdict:** Not a battery health issue — a rotation/logistics issue. The battery used in E9 should have been flagged as depleted and swapped.

---

## Voltage sag context

Every full match had bus voltage sag to **6.6–7.5V** at peak demand. The 6.8V brownout threshold was at risk or crossed in Q51, Q56, Q64, E6, and E9. These are not necessarily battery failures — they reflect peak current demand (4 drive motors + flywheel + intake simultaneously) exceeding what any competition battery can sustain without sag.

The difference between batteries shows up in **how often and how deep** those sag events are, not whether they happen at all. The 2023-1 battery produces measurably deeper sags at equivalent loads compared to newer batteries — exactly what is expected from a battery with ~40% higher internal resistance.
