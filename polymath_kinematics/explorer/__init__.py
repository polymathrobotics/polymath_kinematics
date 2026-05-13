# Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Kinematic Explorer - trajectory simulation and visualization.

This subpackage provides pure functions for trajectory simulation, plotting,
and export. The streamlit UI is in kinematic_explorer_app.py.
"""

from .config import (
    KINEMATIC_EQUATIONS,
    LATTICE_CONFIG,
    TRAJECTORY_EQUATIONS,
    TRAJECTORY_EQUATIONS_RK4,
    LatticeConfig,
)
from .export import trajectories_to_dataframe
from .plotting import (
    plot_analysis,
    plot_articulated_footprint,
    plot_lattice,
    plot_trajectory_with_footprints,
    plot_vehicle_footprint,
    select_symmetric_trajectories,
)
from .simulation import (
    generate_lattice_articulated,
    generate_lattice_bicycle,
    generate_lattice_differential,
    simulate_trajectory,
    simulate_trajectory_euler,
    simulate_trajectory_rk4,
    single_articulated_trajectory,
    single_bicycle_trajectory,
    single_differential_trajectory,
)
from .types import (
    AnyTrajectory,
    ArticulatedTrajectory,
    BicycleTrajectory,
    DifferentialTrajectory,
    Trajectory,
)

__all__ = [
    # Types
    'AnyTrajectory',
    'ArticulatedTrajectory',
    'BicycleTrajectory',
    'DifferentialTrajectory',
    'Trajectory',
    # Config
    'KINEMATIC_EQUATIONS',
    'LATTICE_CONFIG',
    'LatticeConfig',
    'TRAJECTORY_EQUATIONS',
    'TRAJECTORY_EQUATIONS_RK4',
    # Simulation
    'generate_lattice_articulated',
    'generate_lattice_bicycle',
    'generate_lattice_differential',
    'simulate_trajectory',
    'simulate_trajectory_euler',
    'simulate_trajectory_rk4',
    'single_articulated_trajectory',
    'single_bicycle_trajectory',
    'single_differential_trajectory',
    # Plotting
    'plot_analysis',
    'plot_articulated_footprint',
    'plot_lattice',
    'plot_trajectory_with_footprints',
    'plot_vehicle_footprint',
    'select_symmetric_trajectories',
    # Export
    'trajectories_to_dataframe',
]
