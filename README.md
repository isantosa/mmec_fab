# mmec_fab

Starting point for fabrication setup for mmec_fab. Archived to encourage development in own repositories.

Scripts to run fabrication sequences needs to be written, and the methods in mmec_fab.RobotClient needs to be adapted to project needs.

There are some settings hardcoded in `src/mmec_fab/robot_control.py`, like tool,
workobject, speed, acceleration that should be tweaked as well.

## Installation

### Install in conda

```cmd
conda create -n mmec_fab python==3.8 compas_fab==0.17 -y
conda activate mmec_fab
cd /path/to/mmec_fab
pip install -e .
```

### Make accesible in Rhino/Grasshopper

```cmd
conda activate mmec_fab
python -m compas_rhino.install -v 6.0
```

## Usage

### Run docker setup

#### For virtual controller

```cmd
cd /path/to/mmec_fab/docker/virtual-controller
docker-compose up -d
```

Or right-click on file in VS Code to compose up.

#### For real controller

```bash
cd /path/to/mmec_fab/docker/irc5-service-port
docker-compose up -d 
```

Or right-click on file in VS Code to compose up.

### Run fabrication

Can either be used in grasshopper with `Rhino.Geometry` from component inputs
or `compas.geometry` from a json-file.

#### Example script grasshopper

See `examples/pick_place_from_geo_and_json_creation.ghx`.

#### Example script json

First write some frames to json from grasshopper using `examples/pick_place_from_geo_and_json_creation.ghx`.

To run it:

```cmd
conda activate mmec_fab
python /path/to/mmec_fab/examples/pick_place_from_json.py /path/to/json_file.json
```

Or run directly from VS Code, but then you can't set input file.
