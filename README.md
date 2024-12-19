# krecviz
Visualisation utilities for krec files 

![image](https://github.com/user-attachments/assets/9d53e560-f6d4-42d0-a5df-b6ef6aa26ab2)


## Installation

```bash
pip install git+https://github.com/kscalelabs/krecviz.git
# or clone the repo and run
pip install -e .
```

## Usage

NOTE: For now, in the rerun viwer, make sure to select "log tick" as the time unit. will fx this soon

![image](https://github.com/user-attachments/assets/360e1e22-3dbf-4382-b21e-da85174f9206)


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

