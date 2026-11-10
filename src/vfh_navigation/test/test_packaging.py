"""Exercise installation metadata in source and stale colcon build directories."""

from pathlib import Path
import runpy
import shutil
from unittest.mock import patch


def test_stale_build_symlinks_do_not_change_installation_manifest(tmp_path, monkeypatch):
    """Ignore obsolete build artifacts and install only existing package resources."""
    source = Path(__file__).resolve().parents[1]

    def installation_manifest(directory):
        monkeypatch.chdir(directory)
        with patch('setuptools.setup') as setup:
            runpy.run_path(str(directory/'setup.py'), run_name='__main__')
        manifest = setup.call_args.kwargs['data_files']
        for destination, filenames in manifest:
            for filename in filenames:
                assert Path(filename).is_file(), (destination, filename)
        return manifest

    expected = installation_manifest(source)
    shutil.copy2(source/'setup.py', tmp_path/'setup.py')
    for name in ('setup.cfg', 'package.xml', 'README.md', 'VALIDATION.md',
                 'resource', 'vfh_navigation'):
        (tmp_path/name).symlink_to(source/name)
    (tmp_path/'config').mkdir()
    for filename in (source/'config').glob('*.yaml'):
        (tmp_path/'config'/filename.name).symlink_to(filename)
    obsolete = tmp_path/'config'/r'vfh_params_og\.yaml'
    obsolete.symlink_to(tmp_path/'missing_original_config.yaml')
    assert obsolete.is_symlink() and not obsolete.exists()
    assert installation_manifest(tmp_path) == expected
