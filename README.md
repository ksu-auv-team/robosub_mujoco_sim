# AUV Simulator

A [MuJoCo](https://mujoco.readthedocs.io/en/stable/overview.html)-based simulator for flying an AUV (submarine) through a course. Run the viewer to look around the course, or run the sub with keyboard control.

## Requirements

- [uv](https://docs.astral.sh/uv/getting-started/installation/) (manages Python and packages for you)
- [Git](https://git-scm.com/downloads) (on Windows, install Git Bash too)
- Python 3.12 or newer (uv will download it automatically if you don't have it)

Install uv:

```bash
# macOS / Linux
curl -LsSf https://astral.sh/uv/install.sh | sh

# Windows (PowerShell)
powershell -c "irm https://astral.sh/uv/install.ps1 | iex"
```

## Setup

1. Clone this repository and enter the folder:

   ```bash
   git clone <repo-url>
   cd <repo-folder>
   ```

2. Install Python and all dependencies (MuJoCo, NumPy, etc.) from the project's `pyproject.toml`:

   ```bash
   uv sync
   ```

This creates a `.venv` in the repo folder. You don't need to activate it, because `uv run` uses it automatically.

## Usage

### View the course

```bash
uv run python -m mujoco.viewer --mjcf=build/course.xml
```

### Fly the sub with keyboard control

Run these in two separate terminals, both from the repo folder.

Terminal 1 (simulation):

```bash
uv run python fly_auv.py
```

Terminal 2 (keyboard controller):

```bash
uv run python auv_keyboard.py
```

Keep the small popup window focused so your keypresses are registered.

### macOS users

On macOS, MuJoCo's interactive viewer must run under `mjpython` instead of `python`.

**Installing mjpython:** it isn't a separate package. It ships with the `mujoco` pip package on macOS, so running `uv sync` during setup already installs it. To confirm it's there:

```bash
uv run which mjpython
```

This should print a path inside `.venv/bin/`. If nothing prints, `mujoco` is missing or outdated, so reinstall it:

```bash
uv sync --reinstall-package mujoco
```

**Using mjpython:** run it in place of `python` for the script that opens the viewer:

```bash
uv run mjpython fly_auv.py
```

The keyboard controller and the standalone viewer command above run as normal with `uv run python`.

If `mjpython` fails with an error like `dyld: Library not loaded: @rpath/libpython3.12.dylib`, uv's downloaded Python is missing the shared library `mjpython` needs. Use a Homebrew Python instead:

```bash
brew install python@3.12
rm -rf .venv
uv sync --python /opt/homebrew/bin/python3.12
uv run mjpython fly_auv.py
```

On an Intel Mac, the Homebrew path is `/usr/local/bin/python3.12`.

## Troubleshooting

**"The requested interpreter resolved to Python 3.11 ... incompatible with the project's Python requirement"**

You have an old virtual environment built with an older Python. Delete it and re-sync:

```bash
rm -rf sim .venv
uv sync
```

## Contributing

The simulator will continue to be updated. When working on it, please create your own branch:

```bash
git checkout -b your-branch-name
```

If you need to add a package, use `uv add <package>` and let the maintainers know so the change can be reviewed before it goes into the shared `pyproject.toml`.
