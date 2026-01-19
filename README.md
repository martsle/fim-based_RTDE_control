# fim-based_RTDE_control

__fim-based RTDE control__ is a robot control tool that uses a _Fabrication Information Modeling (FIM)_ model as its information backbone. It orchestrates additive manufacturing processes by linking digital fabrication data with real-time robot control and data feedback.

## Prerequesites

The tool requires an existing FIM model and an active connection to a time series database.

The FIM model can be provided in one of the following forms:

- As an IFC file (limited functionality), or

- Preferably as a Neo4j graph database, enabling full access to structured fabrication and task information.

For process monitoring and feedback, a time series database connection is required to store and retrieve sensor and execution data. 

Connection details for all external services are configured via environment variables.

## Getting started
### Environment Setup

We recommend to use the package and project manager [_uv_](https://docs.astral.sh/uv/getting-started/) for package management.
To create a virtual environment and install all necessary packages, run:
```bash
uv sync
```

### Configuration

Next, copy the provided .env template and configure it with the required server connections:

```bash
cp .envTEMPLATE .env
```

Edit the .env file to specify:

- Neo4j database connection (URI, username, password)
- Time series database connection (e.g. host, port, authentication)
- Robot controller connection (IP address and RTDE settings)
- Optional logging and debug parameters

## Running the controller

Once the environment and connections are configured, the controller can be started using:

```bash
uv run python RunPrint.py
```

At runtime, the controller:
- Queries the FIM model for fabrication tasks and toolpaths
- Translates fabrication information into RTDE-compatible robot commands
- Executes the additive manufacturing process
- Collects and stores feedback data in the time series database


## System Architecture (Overview)

- __FIM Model__: 
Provides structured fabrication data, task hierarchies, and geometric information.
- __RTDE Control Layer__:
Streams motion and process commands to the robot controller.
- __Time Series Database__:
Stores sensor readings, execution states, and process feedback for monitoring and analysis.