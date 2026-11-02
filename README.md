# robosub_mujoco_sim
This is the repository for the sub simulator, built in MuJoco.

# How to Install and Run

1) Install Python 3.12+ and Git Bash if on Windows. Make sure Python is added to path in the install.
2) Clone this repository on your local computer.
3) Open your terminal (or Git Bash) and get into the folder you cloned
4) Create a virtual environment. This can be done using 'python -m venv sim'.
5) Activate the virtual environment. From where you created it, run either 'source sim/Scripts/activate' or 'source sim/bin/activate'
6) Run the following line to install Mujoco and other required packages: 'pip install mujoco numpy trimesh scipy pillow matplotlib fast-simplification'
7) Afterwards, run 'python -m mujoco.viewer --mjcf=build/course.xml' to open the course.
8) To run it with the sub and control it, run 'python fly_auv.py' in 1 terminal and in a second terminal, run 'python auv_keyboard.py' and remain focused on the small popup window.

The simulator will continue to be updated. When working on it, please remember to create your own branch. 
