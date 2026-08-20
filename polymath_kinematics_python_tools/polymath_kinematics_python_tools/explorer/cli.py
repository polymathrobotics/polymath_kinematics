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
"""Launcher for the Kinematic Explorer Streamlit app.

Kept out of ``kinematic_explorer_app`` because that module's body *is* the Streamlit script:
importing it from an entry point would run every widget in bare mode before the server starts.
"""

from __future__ import annotations

import pathlib
import subprocess
import sys


def app_path() -> pathlib.Path:
    """Absolute path to the Streamlit app script."""
    return pathlib.Path(__file__).resolve().parent.parent / 'kinematic_explorer_app.py'


def main(argv: list[str] | None = None) -> int:
    """Run ``streamlit run kinematic_explorer_app.py``, forwarding any extra arguments.

    Extra arguments go to Streamlit, so e.g. ``kinematic-explorer --server.port=8600`` works.
    """
    args = list(sys.argv[1:] if argv is None else argv)

    script = app_path()
    if not script.is_file():
        print(f'error: could not locate the explorer app at {script}', file=sys.stderr)
        return 1

    command = [
        sys.executable,
        '-m',
        'streamlit',
        'run',
        str(script),
        '--browser.gatherUsageStats=false',
    ]
    # Default to loopback; pass --server.address=0.0.0.0 to serve over the network.
    if not any(arg.startswith('--server.address') for arg in args):
        command.append('--server.address=localhost')
    command.extend(args)
    try:
        return subprocess.call(command)
    except FileNotFoundError:
        print(
            'error: streamlit is not installed. Install this project with:\n  uv sync',
            file=sys.stderr,
        )
        return 1


if __name__ == '__main__':
    sys.exit(main())
