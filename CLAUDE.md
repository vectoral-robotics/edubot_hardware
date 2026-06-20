# edubot_hardware — Claude guidelines

ROS2 package providing the EduBot hardware interface: the serial bridge to the
firmware, mecanum kinematics, odometry, and a simulation interface. Part of the
EduBot ROS2 stack, consumed by `edubot_dashboard` via its vcstool manifest.

These guidelines will grow over time. For now the most important rule:

## Commits

All commits MUST follow the [Conventional Commits](https://www.conventionalcommits.org) spec.

Format:

    <type>(<optional scope>): <short summary>

Common types: `feat`, `fix`, `docs`, `refactor`, `perf`, `test`, `build`, `ci`, `chore`, `revert`.

- Imperative mood ("add", not "added").
- Summary under ~72 characters, lower case, no trailing period.
- Scope is optional and names the affected area.

Example:

    fix(odometry): use correct ticks-per-rev constant
