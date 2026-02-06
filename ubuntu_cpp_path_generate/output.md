# Console Output Reference

This document explains what the C++ CLI prints to the console and why.

## Streams

- stderr: runtime logs, per-step trajectory lines, timing, and errors.
- stdout: the JSON plan payload (only when --output is not provided).

## Per-step trajectory lines (stderr)

When planning succeeds, the program prints one line per trajectory step:

```
[step k] pos=(x, y, z) psi_deg=... theta_deg=... l_rope_m=...
```

- k: zero-based trajectory index.
- pos: hook position (meters), formatted with 3 decimal places.
- psi_deg: slew/yaw angle (degrees).
- theta_deg: luff angle (degrees).
- l_rope_m: rope length (meters).

The sequence of lines follows the same order as payload.trajectory in the JSON output.

## Timing line (stderr)

After planning completes (success or failure), the CLI prints:

```
[timing] planning_ms=...
```

- planning_ms: total planning time in milliseconds (double, 3 decimals).
- The timing includes JSON parsing and planning, but not output file writing.

## Failure line (stderr)

When planning fails, the CLI prints a single line before timing:

```
[failure] <reason>
```

- <reason> matches payload.reason in the JSON output.

## Informational lines (stderr)

- If --input is omitted and examples/input_basic.json is found:
  - [info] --input not provided; using default: <path>
- If --gui is provided:
  - [info] --gui requested; GUI is not supported in the C++ version. Running headless.

## JSON output (stdout or file)

- If --output is NOT provided: the full JSON payload is printed to stdout.
- If --output IS provided: the JSON payload is written to that file and not printed.

The JSON payload is compatible with the Python version and contains:

- status, mode, actions, trajectory, waypoints
- start_config, goal_config

Trajectory entries include k, pos, psi_deg, theta_deg, l_rope_m, and nearest-distance fields.
