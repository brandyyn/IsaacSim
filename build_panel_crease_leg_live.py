import runpy
import os
from pathlib import Path

project_root = Path(__file__).resolve().parent
if not (project_root / 'build_stable_panel_crease_leg.py').exists():
    project_root = Path(os.environ.get('PANEL_CREASE_PROJECT_ROOT', Path.cwd())).expanduser().resolve()
module = runpy.run_path(str(project_root / 'build_stable_panel_crease_leg.py'), run_name='stable_panel_crease_leg_builder')
module['build'](
    project_root / 'input_improved.usd',
    project_root / 'input_improved.json',
    project_root / 'panel_crease_leg_v8.usd',
)
