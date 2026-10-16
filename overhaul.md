# SimSync UI Overhaul

Scope: `web/index.html` (Dashboard + Motor Tuning), `web/guide.html` (Calibration Guide),
`web/css/app.css`, `web/css/guide.css`, `web/js/tuning.js`, `web/js/app.js`.
`webstatic/web/` is a mirror. Copy it after each change, or better, generate it in the build (see §9).

Goal: a calm, professional tool where the next action is always obvious.

---

## 1. Problems found

### 1.1 Visual / theme
| Issue | Where |
|---|---|
| Two unrelated design systems. `app.css` uses neon `#00e5ff / #39ff6b / #ff3d5a` on `#07090f`. `guide.css` uses pastel `#50e4ec / #8cdbb0` on `#080c13` | `app.css:1-29`, `guide.css:1` |
| Fonts differ. The dashboard asks for `Inter` / `JetBrains Mono` but never loads them, so it falls back to browser defaults. The guide uses `system-ui` | `app.css:28,37`, `index.html` `<head>` |
| Neon saturated accents (cyan, purple, amber, green, red, blue, teal) all compete. Seven accent colours, no hierarchy | `app.css:9-16` |
| `--text-muted: #4a5568` on `#0d1117` fails WCAG contrast. 9–11px text is everywhere | `app.css:20`, inline `font-size:9px` |
| Many inline `style=""` attributes (selects, inputs, legend swatches, margins). They bypass the theme | `index.html:61,69,101,237,311,317,409-417` |
| Button colours carry no meaning. Purple for "-10 counts", cyan for "+10", purple for a preset | `index.html:321-325,371` |
| Shouty uppercase labels (`LATCH PAUSE`, `RESUME`, `CLEAR PAUSE`, `GO`, `CENTER`) are mixed with sentence case | header, sidebar |

### 1.2 Motor Tuning layout
- **Duplicated controls.** There are two step-test launchers (sidebar `#stepPanel` and `Test + step / Test − step`), two CSV exports, two "run only this motor" buttons (`#tuningEnable` and `#motorEnable`), two pause buttons (`LATCH PAUSE` and `PAUSE`), plus `CLEAR PAUSE`. There are also two motor selectors: scope joint tabs and `#tuningMotors`, plus `#manJoint` and `#stepJoint` dropdowns.
- **No flow.** Filters, PID, limits, timing, output mode, the stability ladder, the PD sequence, the boundary model, results, position, jog, mapping and display range all appear at once in an unordered grid. Nothing tells the user which step comes first.
- **Instructions are mixed into the controls.** Long `control-note` paragraphs sit under almost every card (`index.html:144-145,155,157,181,188-190,206,343,392,465`). Users have to read essays to press a button.
- **Settings in the wrong place.** Motion Mapping and Display range are game/display settings, not motor tuning, yet they live in the tuning sidebar.
- **Expert controls up front.** Loop timing (Ks, Fpid, Fpwm) is a rarely changed, risky setting. It sits at the same level as Kp.

### 1.3 Calibration Guide
- It is a separate page that sends no commands. Users read the steps there, then go back and hunt for the matching controls.
- Each step is a dense paragraph with 4–6 instructions inside.
- The marketing-style hero ("Smooth motion. One small change at a time.") pushes the content below the fold.
- The guide and the tuning page repeat each other with slightly different wording (stability ladder vs. step 04, boundary model vs. step 02).

### 1.4 Copy
Verbose, hedging, jargon-first. Examples:
- "Solo mode stops the other motors and enables manual/test output for one motor. Pause stops it. Resume returns to all included motors."
- "Human-readable estimate. Values outside the pot range are extrapolated and may be unreliable. Position commands, steps, host limits and CSV always use counts. This range does not restrict motor travel."

---

## 2. Design principles
1. **One task per screen.** Each view answers one question: "what do I do now?"
2. **Steps, not cards.** Calibration and tuning become a guided, numbered wizard.
3. **Control and explanation live together, briefly.** Show a one-line hint. Put the details in a `ⓘ` popover.
4. **Every control exists once.**
5. **Colour means state.** Use neutral by default, one brand accent, and reserve green/amber/red for OK, caution and danger.
6. **Progressive disclosure.** Hide advanced settings behind a single "Advanced" disclosure.

---

## 3. New information architecture

```
Header: SimSync · [Serial ●] [Telemetry ●] · [Motors: Paused ▾]   [■ STOP]
Tabs:   Drive | Setup | Tune | Settings
```

| Tab | Purpose | Contents |
|---|---|---|
| **Drive** | Normal use | Status, scope (read-only), joint cards, game telemetry, Resume/Pause |
| **Setup** (replaces guide.html) | First-time calibration wizard | Steps 1–4 below, with live controls inside each step |
| **Tune** | Per-motor PID | Motor picker, scope, step test, gains, results |
| **Settings** | Rarely changed | Motion mapping, display units/range, filters, loop timing (Advanced), EEPROM, import/export |

`guide.html` becomes a short read-only **Reference** (filters, PID terms, troubleshooting), linked from `ⓘ` popovers. It no longer repeats the procedure.

### 3.1 Global safety model (one control set)
- One header **STOP** button (red, always visible, `Space`/`Esc` hotkey). It latches.
- One status chip: `Paused` / `Running` / `Solo · M2` / `Stopped`. Clicking it opens Resume / Pause / Clear stop.
- Remove `#disarmBtn`, `#estopClearBtn`, `#soloStop` and the duplicate `LATCH PAUSE` wording.

---

## 4. Setup wizard (replaces the Calibration Guide)

A left stepper with a content pane on the right. A step is marked complete when its check passes, verified from live readback where possible.

| # | Step | Controls shown | Done when |
|---|---|---|---|
| 1 | **Connect** | Serial port picker, firmware version, `Bench mode (no game)` toggle | Readback live, motors paused |
| 2 | **Set travel** (per motor M1/M2/M3 sub-tabs) | Live pot value, buttons `Set min` · `Set center` · `Set max`, a visual range bar showing cutoff ▸ clip ▸ host bounds | Bounds valid (auto-checked: cutoff < clip, host ≥ 4 inside clip) |
| 3 | **Check direction** | Jog `◀` / `▶` (low PWM, short pulse), live feedback arrow | User confirms "Feedback moved the same way" for each motor |
| 4 | **First tune** | Hands off to the Tune tab, pre-selecting the first untuned motor | All motors have a saved tune |

Each step: a title, **one** sentence, the controls, a pass condition, and `Next`.
The safety notice ("Support the rig. Keep a power cut-off within reach.") shows once, as a dismissible banner on step 1.

Replace the boundary paragraphs with the **range bar** visual:
```
0 ─┤cutoff├──┤clip├──[ host min ●center host max ]──┤clip├──┤cutoff├─ 1023
```
Invalid zones turn red with an inline message, e.g. "Host max must be ≤ 979."

---

## 5. Tune tab

Layout (desktop):
```
┌ Motor: [M1] [M2] [M3]  ·  Solo ○ ───────────────────────────────┐
│ Scope (target / actual / error / PWM)   5s 10s 30s   ⏸  ⤓      │
├──────────────────────────────┬──────────────────────────────────┤
│ 1 Step test                  │ 2 Gains                          │
│  Size [ 8 ] counts           │  Kp ━━━●━━ [20.0]                │
│  [Step +]  [Step −]          │  Ki ●━━━━━ [0.00]                │
│  Result: rise 42ms · OS 6%   │  Kd ━●━━━━ [0.40]                │
│  Verdict: ✓ Settled          │  [Apply]  Revert · Use best      │
├──────────────────────────────┴──────────────────────────────────┤
│ 3 History  (table of runs, compare, export CSV)                  │
├──────────────────────────────────────────────────────────────────┤
│ ▸ Limits (PWM max/min/reverse, deadzone)                         │
│ ▸ Signal test (sine/triangle/square)                             │
└──────────────────────────────────────────────────────────────────┘
```
Changes:
- One motor selector drives everything. Delete `#manJoint`, `#stepJoint`, and the scope joint tabs on this view.
- One step-test control. Merge `#stepPanel` with `Test ±`.
- Gains show **staged vs. live** with a diff badge (`Kp 20.0 → 21.0`). `Apply` sends the staged values, `Revert` discards them.
- Show a **verdict chip** after each step: `✓ Settled`, `⚠ Overshoot 18%`, `⚠ PWM saturated`, `✕ Did not settle`. It replaces the free-text `#tuningAdvice`.
- Coaching hint (single line, contextual), derived from the verdict:
  - Overshoot → "Add a little Kd, then step again."
  - Slow → "Raise Kp ~5%, then step again."
  - Saturated → "PWM is capped. Lower Kp, or raise PWM max by 5–10."
- Delete the stability ladder and PD sequence blocks. Their content becomes these hints plus the Reference page.
- Move Filters and Loop timing to Settings → Advanced.
- Move Position / Jog to Setup step 3. Keep a small `Nudge ±5` on Tune only.

---

## 6. Design system (shared `web/css/tokens.css`)

Both pages import one file. Delete the `:root` blocks in `app.css` and `guide.css`.

```css
:root {
  color-scheme: dark;
  /* surfaces — neutral slate, not blue-black */
  --bg:        hsl(220 14% 8%);
  --surface-1: hsl(220 13% 11%);
  --surface-2: hsl(220 12% 15%);
  --border:    hsl(220 10% 22%);
  /* text */
  --text:      hsl(220 15% 92%);
  --text-2:    hsl(220 10% 68%);   /* ≥ 4.5:1 on surface-1 */
  --text-3:    hsl(220 8% 52%);    /* labels only, ≥ 3:1 */
  /* one brand accent */
  --accent:    hsl(190 80% 55%);
  --accent-bg: hsl(190 80% 55% / .12);
  /* semantic only */
  --ok:     hsl(150 55% 50%);
  --warn:   hsl(38 90% 58%);
  --danger: hsl(355 75% 60%);
  /* plot traces (scope only) */
  --trace-target: var(--accent);
  --trace-actual: hsl(265 70% 72%);
  --trace-error:  var(--warn);
  --trace-pwm:    hsl(150 40% 60%);

  --font: 'Inter', system-ui, sans-serif;
  --mono: 'JetBrains Mono', ui-monospace, monospace;
  --fs-xs: 12px; --fs-sm: 13px; --fs-md: 15px; --fs-lg: 18px; --fs-xl: 24px;
  --r-sm: 6px; --r-md: 10px;
  --space: 8px;
}
```
Rules:
- Minimum font size **12px**. Use mono only for numbers.
- Buttons: `primary` (accent fill), `secondary` (outline), `danger` (red, only for Stop). Delete `btn-purple`, `btn-cyan`, `btn-amber`, `btn-green`.
- Sentence case everywhere except the `STOP` button.
- Ship the Inter and JetBrains Mono woff2 files in `web/fonts/` and `@font-face` them locally. The app runs offline, so no CDN.
- Drop the neon glows. Use one subtle border plus `surface` elevation instead.
- Remove every inline `style=""`. Scope legend colours come from tokens, and `plots.js` reads them via `getComputedStyle`.
- Optional: a light theme via `[data-theme=light]` that overrides the surface and text tokens only.

---

## 7. Copy rewrite

Rules: verb-first. Use one sentence and no more than about 12 words for inline hints. Put details in the `ⓘ` popover. Use "motor" (not joint/channel/axis) and "counts" consistently.

| Before | After |
|---|---|
| LATCH PAUSE / PAUSE / CLEAR PAUSE / RESUME | **STOP** · status chip → *Resume* / *Pause* |
| Run only selected motor / Run only this motor | **Solo** toggle |
| Stage best measured gains | **Use best** |
| Solo mode stops the other motors and enables… | "Other motors are held while solo is on." |
| Compare the same step size and direction… | *(delete; covered by verdict)* |
| LPF smooths the feedback used by PID. Lower cutoff adds… | "Smooths feedback. Lower = smoother but slower. ⓘ" |
| Pause and turn jog mode off to apply… | Disable `Apply` with tooltip "Pause motors to apply." |
| Human-readable estimate. Values outside the pot range… | "Display only. Doesn't limit travel." |
| Pause to apply limits. Margins are symmetric… | Replaced by range-bar validation messages |
| Check one unloaded motor at a time: a short positive jog… | "Press ▶. Feedback should go up." |
| Smooth motion. One small change at a time. (hero) | *(delete)* |
| Waiting for controller (×6) | Show it once, in the header chip: "Connecting…" |

**Rule for state text:** show disabled-state reasons as tooltips on the disabled control, not as paragraphs.

---

## 8. Implementation phases

| Phase | Work | Files |
|---|---|---|
| **P1 Tokens & cleanup** | Add `tokens.css` and local fonts. Remove inline styles. Unify button classes. Fix contrast. | `css/*`, `index.html`, `guide.html`, `plots.js` |
| **P2 Safety bar** | Single STOP plus status chip. Remove duplicate pause buttons. Keyboard shortcut. | `index.html`, `app.js` |
| **P3 Tab restructure** | Drive / Setup / Tune / Settings. Move Mapping, Display and Advanced to Settings. | `index.html`, `app.js`, `tuning.js` |
| **P4 Tune tab** | Single motor selector, merged step test, staged-vs-live gains, verdict chip, hints. Delete the ladder and PD blocks. | `tuning.js`, `index.html` |
| **P5 Setup wizard** | Stepper, range bar, direction check, completion state persisted in settings | new `js/setup.js`, `index.html` |
| **P6 Reference & copy** | Shrink `guide.html` to a reference. Apply the copy table. Add `ⓘ` popovers. | `guide.html`, all |
| **P7 QA** | Test keyboard navigation, contrast audit (axe), 1280px and 900px widths, and existing Go tests (`go test ./...`). | — |

Keep all element IDs that `app.js` and `tuning.js` bind to, or update them in the same commit. Grep each ID before deleting it.

---

## 9. Build hygiene
- Stop hand-copying `web/` → `webstatic/web/`. Either point `//go:embed` at `web/` directly (move `embed.go` next to it), or add a copy step to `build.sh` / `build.ps1` and gitignore the mirror.

## 10. Done criteria
- [ ] A new user can go from connected to first settled step test without opening another page.
- [ ] No control is duplicated. No inline styles remain. One `:root`.
- [ ] No paragraph over 20 words is visible by default on Tune or Setup.
- [ ] All text meets WCAG AA. Minimum font is 12px.
- [ ] STOP is reachable from every view and by keyboard.
