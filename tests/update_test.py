"""Exercise the installed updater with the original installer and mocked downloads.

No network, ROS install, real service, home directory or credentials are touched.
"""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest

CLI, INSTALLER = map(Path, sys.argv[1:3])
sys.argv = [sys.argv[0]]
with tempfile.TemporaryDirectory(prefix='rearguard-version-test-') as state:
    CURRENT = subprocess.check_output([str(CLI), '--version'], text=True,
        env={**os.environ, 'REARGUARD_STATE_DIR': state}).strip().split()[-1]


def executable(path, content):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content)
    path.chmod(0o755)


class UpdateTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='rearguard update test ')
        self.root = Path(self.tmp.name)
        self.prefix = self.root / 'custom installation'
        (self.prefix / 'bin').mkdir(parents=True)
        shutil.copy2(CLI, self.prefix / 'bin/rearguard')
        target = self.prefix / 'share/harness/install/install.sh'
        target.parent.mkdir(parents=True)
        shutil.copy2(INSTALLER, target)
        self.home = self.root / 'home'
        self.home.mkdir()
        self.state = self.root / 'state'
        self.state.mkdir()
        self.recording = self.state / 'recording.mcap'
        self.recording.write_bytes(b'keep this incident')
        self.credential = self.home / '.config/rearguard/device.json'
        self.credential.parent.mkdir(parents=True)
        self.credential.write_text('{"test": "keep pairing"}')
        self.fakebin = self.root / 'fakebin'
        executable(self.fakebin / 'uname', '#!/bin/sh\ncase "$1" in -s) echo Linux;; -m) echo x86_64;; esac\n')
        executable(self.fakebin / 'curl', '''#!/usr/bin/env python3
import json, os, shutil, sys
from pathlib import Path
args=sys.argv[1:]
out=Path(args[args.index('-o')+1])
url=next(a for a in args if a.startswith('https://'))
with open(os.environ['TEST_DOWNLOAD_LOG'],'a') as log: log.write(url+'\\n')
root=Path(os.environ['TEST_FIXTURES'])
shutil.copyfile(root/('manifest.json' if url.endswith('.json') else 'release.tar.gz'),out)
''')
        self.env = {**os.environ, 'HOME': str(self.home), 'REARGUARD_STATE_DIR': str(self.state),
                    'XDG_CONFIG_HOME': str(self.home / '.config'), 'SHELL': '/bin/sh',
                    'PATH': str(self.fakebin) + os.pathsep + os.environ['PATH'],
                    'TEST_DOWNLOAD_LOG': str(self.root / 'downloads'),
                    'TEST_INSTALL_LOG': str(self.root / 'installed-actions'), 'TEST_FIXTURES': str(self.root),
                    # Caller overrides must not redirect updating another installation.
                    'HARNESS_INSTALL_DIR': str(self.root / 'wrong-installation')}
        self.artifact('9.8.7')

    def tearDown(self):
        self.tmp.cleanup()

    def artifact(self, version, bad_hash=False):
        package = self.root / ('harness-kernel-' + version)
        executable(package / 'bin/rearguard', '''#!/bin/sh
printf '%s\\n' "$*" >> "$TEST_INSTALL_LOG"
case "$1" in
 --version) echo rearguard VERSION;;
 abi) echo '{}';;
 service) case "$2" in status) echo '{"running":true}';; esac;;
esac
'''.replace('VERSION', version))
        executable(package / 'bin/harness_critic', '#!/bin/sh\nexit 2\n')
        executable(package / 'share/harness/install/install_runtime_deps.sh', '#!/bin/sh\nexit 0\n')
        # A full upgrade replaces the bundled updater while its copied script runs.
        shutil.copy2(INSTALLER, package / 'share/harness/install/install.sh')
        (package / 'share/harness/runtime').mkdir(parents=True, exist_ok=True)
        (package / 'share/harness/runtime/edge-runtime.json').write_text('{"critic_activation":"after_profile_setup"}')
        archive = self.root / 'release.tar.gz'
        with tarfile.open(archive, 'w:gz') as tar:
            tar.add(package, arcname=package.name)
        sha = '0'*64 if bad_hash else hashlib.sha256(archive.read_bytes()).hexdigest()
        (self.root / 'manifest.json').write_text(json.dumps({'version': version,
            'artifacts': {'kernel_tarball': {'path': f'harness-kernel-{version}-x86_64-linux-gnu.tar.gz', 'sha256': sha}}}))

    def run_cli(self, *args):
        return subprocess.run([str(self.prefix / 'bin/rearguard'), 'update', *args], env=self.env,
                              capture_output=True, text=True, timeout=30)

    def preserved(self):
        self.assertEqual(self.recording.read_bytes(), b'keep this incident')
        self.assertEqual(self.credential.read_text(), '{"test": "keep pairing"}')
        self.assertFalse((self.root / 'wrong-installation').exists())

    def test_check_only_does_not_replace_or_restart(self):
        before = (self.prefix / 'bin/rearguard').read_bytes()
        result = self.run_cli('--check')
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
        self.assertIn('Available: 9.8.7', result.stdout)
        self.assertEqual((self.prefix / 'bin/rearguard').read_bytes(), before)
        self.assertEqual(len((self.root / 'downloads').read_text().splitlines()), 1)
        self.assertFalse((self.root / 'installed-actions').exists())
        self.preserved()

    def test_update_preserves_data_uses_current_prefix_and_restarts(self):
        result = self.run_cli()
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
        actions = (self.root / 'installed-actions').read_text()
        self.assertIn('service start\n', actions)
        self.assertIn('service status\n', actions)
        self.assertIn('/releases/download/v9.8.7/', (self.root / 'downloads').read_text())
        self.preserved()

    def test_same_version_is_a_noop(self):
        self.artifact(CURRENT)
        result = self.run_cli()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('already installed', result.stdout)
        self.assertEqual(len((self.root / 'downloads').read_text().splitlines()), 1)
        self.assertFalse((self.root / 'installed-actions').exists())
        self.preserved()

    def test_checksum_failure_preserves_existing_install(self):
        self.artifact('9.8.7', bad_hash=True)
        before = (self.prefix / 'bin/rearguard').read_bytes()
        result = self.run_cli('--version', '9.8.7')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('sha256 mismatch', result.stderr)
        self.assertEqual((self.prefix / 'bin/rearguard').read_bytes(), before)
        self.assertFalse((self.root / 'installed-actions').exists())
        self.preserved()

    def test_missing_bundled_installer_refuses_update(self):
        (self.prefix / 'share/harness/install/install.sh').unlink()
        result = self.run_cli()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('original installer', result.stderr)
        self.assertFalse((self.root / 'downloads').exists())
        self.preserved()

    def test_bad_arguments_never_download(self):
        for args in (('--version', '../../bad'), ('--version', '1.2.3;exit'), ('--typo',), ('--version',)):
            self.assertNotEqual(self.run_cli(*args).returncode, 0)
        self.assertFalse((self.root / 'downloads').exists())
        self.preserved()


if __name__ == '__main__':
    unittest.main()
