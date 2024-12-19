# krecviz
Visualisation utilities for krec files 

## Installation

```bash
pip install git+https://github.com/kscalelabs/krecviz.git
# or clone the repo and run
pip install -e .
```

## Usage

NOTE: In the rerun viwer, make sure to select "log tick" as the time unit.

CLI usage:

```bash
# cd to the repo root
cd krecviz
python visualize.py --urdf data/urdf_examples/gpr/robot.urdf --krec data/krec_examples/actuator_22_right_arm_shoulder_roll_movement.krec --output output.rrd
```

Python API usage:

```python
import krecviz

krecviz.viz(
    krec_path="path/to/recording.krec",
    urdf_path="path/to/robot.urdf"
)
```

