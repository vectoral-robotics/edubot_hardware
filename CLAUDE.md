# edubot_hardware — Claude guidelines

ROS 2 package providing the EduBot hardware interface: the serial bridge to the
firmware, mecanum kinematics, odometry, and a simulation backend. Part of the
EduBot ROS 2 stack, consumed by `edubot_bringup`.

This is the **reference repo** for the EduBot ROS stack: conventions and tooling
established here are replicated to the other repos.

## Language

Everything is written in **English** — code, comments, docstrings, README,
commit messages, config comments, identifiers. This holds even when a chat with
the maintainer is in another language.

## Naming: OmniBot → EduBot

The project was formerly called **OmniBot**; it is now **EduBot**. Always use
`EduBot`/`edubot`. If any `OmniBot`/`omnibot` leftovers turn up, fix them.

## Commits

All commits MUST follow the [Conventional Commits](https://www.conventionalcommits.org) spec.
Enforced by the `commitizen` commit-msg hook.

Format:

    <type>(<optional scope>): <short summary>

Common types: `feat`, `fix`, `docs`, `refactor`, `perf`, `test`, `build`, `ci`, `chore`, `revert`.

- Imperative mood ("add", not "added").
- Summary under ~72 characters, lower case, no trailing period.
- Scope is optional and names the affected area.

Example: `fix(odometry): use correct ticks-per-rev constant`

## Metadata

- Maintainer / contact: **Vectoral**, **info@vectoral.ch** (in `package.xml` and
  `setup.py`). Author attribution may stay personal.
- License: **PolyForm Perimeter 1.0.0** — keep `LICENSE`, `package.xml`,
  `setup.py` and README consistent.

## Development environment

Dev tooling is managed with **uv** (the ROS build itself stays
`colcon`/`ament_python` via `setup.py`):

```bash
uv sync                                         # create .venv with dev tools
uv run pre-commit install --install-hooks       # git hooks (once per clone)
uv run pre-commit install --hook-type commit-msg
```

Config lives in `pyproject.toml`: ruff (`[tool.ruff]`), pytest+coverage,
commitizen. The dev env pins Python **3.10** (Humble) via `.python-version`.

Note: a machine without ROS (e.g. dev laptop) can run ruff, pytest and
commitizen, but not anything importing `rclpy` (the node) — that runs on the
robot or in the CI container.

## Linting, formatting, tests

Before every push (pre-commit does 1–2 automatically):

```bash
uv run ruff check --fix .
uv run ruff format .
uv run pytest                 # test suite (bare; works with colcon test too)
uv run pytest --cov=edubot_hardware --cov-report=term-missing   # with coverage
```

- ruff rule sets: `E,F,W,I,B,UP,SIM,RUF`, `ignore = ["E501"]`, line length 99.
- Tests live in `test/` (ament convention) and cover the pure-logic modules
  (kinematics, odometry, simulation, serial parser). The `rclpy`-bound
  `hardware_node` is exercised via ROS integration, not unit tests.

## Versioning & releases

`commitizen` derives the next version from the commit history and bumps it in
both `package.xml` and `setup.py`, and updates `CHANGELOG.md`:

```bash
uv run cz bump            # creates the version tag + changelog at release time
```

## Architecture (orientation)

The package is split into small, testable pieces:

- `hardware_node.py` — the ROS 2 node (parameters, topics, timers, QoS)
- `mecanum_kinematics.py` — forward/inverse mecanum kinematics (pure)
- `odometry.py` — encoder-tick → pose/velocity integration (pure)
- `serial_interface.py` — serial bridge to the ESP32
- `simulation_interface.py` — protocol-compatible simulator (no hardware)

ESP32 protocol — TX: `M w_rr w_fr w_rl w_fl` (wheel rad/s);
RX: `E seq timestamp_us t_rr t_fr t_rl t_fl` (cumulative encoder ticks).
Wheel order is always **RR, FR, RL, FL**.
