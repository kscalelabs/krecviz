# krecviz
Visualisation utilities for krec files

![image](https://github.com/user-attachments/assets/9d53e560-f6d4-42d0-a5df-b6ef6aa26ab2)

https://github.com/user-attachments/assets/0441c859-ab77-4eec-9b76-083b52f077e9

## Installation

### Python

```bash
pip install git+https://github.com/kscalelabs/krecviz.git
# or clone the repo and run
pip install -e .
```

### Rust

#### Install Protocol Buffers

First, make sure you have protobufs installed for the krec dependency. Follow these commands to install it:

```bash
cd /tmp
wget https://github.com/protocolbuffers/protobuf/releases/download/v28.3/protoc-28.3-linux-x86_64.zip
unzip protoc-28.3-linux-x86_64.zip
sudo cp bin/protoc /usr/local/bin/protoc
``` 

#### Build the krecviz Rust library

```bash
# cd to the repo root
cd krecviz_rust
cargo build
```

## Usage

NOTE: For now, in the rerun viwer, make sure to select "log tick" as the time unit. will fx this soon

![image](https://github.com/user-attachments/assets/360e1e22-3dbf-4382-b21e-da85174f9206)

### Python

CLI usage:

```bash
# cd to the repo root
cd krecviz

python -m krecviz.visualize \
    --urdf ../tests/assets/urdf_examples/gpr/robot.urdf \
    --krec ../tests/assets/krec_examples/actuator_22_right_arm_shoulder_roll_movement.krec
```

Python API usage:

```python
import krecviz

krecviz.viz(
    krec_path="path/to/recording.krec",
    urdf_path="path/to/robot.urdf"
)
```

### Rust

```bash
# cd to the repo root
cd krecviz_rust
cargo run -- \
    --urdf ../tests/assets/urdf_examples/gpr/robot.urdf \
    --krec ../tests/assets/krec_examples/actuator_22_right_arm_shoulder_roll_movement.krec


# run in debug mode 
RUST_LOG=krecviz_rust=debug  cargo run --    \
    --urdf ../tests/assets/urdf_examples/gpr/robot.urdf \
    --krec ../tests/assets/krec_examples/actuator_22_right_arm_shoulder_roll_movement.krec
```

## Tests

### Python 

These tests will show what a correct visualization should look like.

In the `krecviz/tests/test_visualize.py` file, you can uncommnet which visualization test you want to run. (In python, running multiple visualizations from the same file overwrites the previous visualization, so one at a time for now.)

```bash
# cd to the repo root
cd krecviz
python -m tests.test_visualize 
```

### Rust

These tests will show incorrect rotations, despite having the same logic as the python code/tests.

```bash
# cd to the repo root
cd krecviz_rust
cargo test
```
