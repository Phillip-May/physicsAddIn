# Contributing

Keep changes small enough to review and test them with the narrowest relevant smoke before running
the full release gates in [docs/testing.md](docs/testing.md).

## Code style

- Use the repository `.clang-format` for new or substantially edited C++ code. Do not reformat an
  unrelated file as part of a functional change.
- Prefer a small concrete type over a speculative interface. Add an abstraction when there are at
  least two real consumers with a shared contract.
- Put host-independent behavior in `Common/`; keep Qt, ImGui, and RoboDK integration in their host
  directories.
- Avoid new global state. Pass the narrow dependency a component needs.

## Comments

Comments should preserve information that the code cannot express: coordinate conventions, ownership,
threading requirements, external API quirks, and the reason behind a surprising constraint.

Do not narrate the next statement, defend routine implementation choices, record refactoring history,
or describe tests that already state the behavior. Put broader design context in `docs/` and keep API
comments short.
