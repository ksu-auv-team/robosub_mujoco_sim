# AUV Simulator

A MuJoCo-based simulator for flying an AUV (submarine) through a course. Run the viewer to look around the course, or run the sub with keyboard control.

1) Install Python 3.12+ and Git Bash if on Windows. Make sure Python is added to path in the install.
2) Clone this repository on your local computer.
3) Open your terminal (or Git Bash) and get into the folder you cloned
4) Create a virtual environment. This can be done using 'python -m venv sim'.
5) Activate the virtual environment. From where you created it, run either 'source sim/Scripts/activate' or 'source sim/bin/activate'
6) Run the following line to install Mujoco and other required packages: 'pip install mujoco numpy trimesh scipy pillow matplotlib fast-simplification'
7) Afterwards, run 'python -m mujoco.viewer --mjcf=build/course.xml' to open the course.
8) To run it with the sub and control it, run 'python fly_auv.py' in 1 terminal and in a second terminal, run 'python auv_keyboard.py' and remain focused on the small popup window.

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