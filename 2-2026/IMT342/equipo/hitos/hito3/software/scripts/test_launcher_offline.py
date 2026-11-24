"""Pruebas de los scripts originales con dobles locales, sin red ni llavero."""
import os
from pathlib import Path
import shutil
import subprocess
import sys
from tempfile import TemporaryDirectory
import unittest


SCRIPTS = Path(__file__).resolve().parent
FIXTURE_YAML = """\
imt342_coordinator:
  ros__parameters:
    rws_url: "http://192.0.2.1"
    rws_task: "T_ROB1"
    rws_module: "Imt342JointServer"
    rws_mechunit: "ROB_1"
    rws_tolerance_rad: 0.005
"""


class LauncherTest(unittest.TestCase):
    def setUp(self):
        scratch = TemporaryDirectory(prefix=".launcher-test-", dir=SCRIPTS.parent)
        self.addCleanup(scratch.cleanup)
        root = Path(scratch.name)
        self.scripts = root / "scripts"
        self.bin = root / "bin"
        self.scripts.mkdir()
        self.bin.mkdir()
        for name in ("imt342_launcher.sh", "irc5_demo.sh", "irc5_probe.py"):
            shutil.copy2(SCRIPTS / name, self.scripts / name)
        self.config = root / "irc5.yaml"
        self.config.write_text(FIXTURE_YAML, encoding="utf-8")
        self.trace = root / "trace"
        self.env = {
            **{name: value for name, value in os.environ.items() if not name.startswith("IMT342_")},
            "PATH": f"{self.bin}:/usr/bin:/bin",
            "IMT342_RWS_KEYRING_ONLY": "1",
            "TEST_TRACE": str(self.trace),
            "TEST_ACCOUNT": "available",
        }

    def stub(self, name, body, *, folder=None, interpreter="/usr/bin/bash"):
        script = (folder or self.bin) / name
        script.write_text(f"#!{interpreter}\n{body}\n", encoding="utf-8")
        script.chmod(0o700)

    def run_script(self, name, *args):
        return subprocess.run([self.scripts / name, *args], env=self.env, capture_output=True,
                              text=True, timeout=10, check=False)

    def test_protected_launcher_dispatches_offline(self):
        self.stub("zenity", """
[[ "$1" == --list ]] || exit 91
printf 'Offline\n'
""")
        self.stub("rviz_fedora.sh", """
if [[ "$*" == 'status --session' ]]; then
    exit 3
fi
[[ "$*" == offline && "$IMT342_RWS_KEYRING_ONLY" == 1 ]] || exit 92
printf '%s\n' offline >"$TEST_TRACE"
""", folder=self.scripts)
        result = self.run_script("imt342_launcher.sh")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.trace.read_text().strip(), "offline")

    def stub_account_dependencies(self):
        self.stub("python3", f"""
if [[ "$1" == - && "$#" == 3 ]]; then
    [[ "$2" == 192.0.2.1 && "$3" == 80 ]] || exit 93
    cat >/dev/null
    printf 'TCP simulado\n'
    exit 0
fi
exec {shutil.which("python3", path="/usr/bin:/bin")} "$@"
""")
        self.stub("ip", """
case "$*" in
    '-j -4 route get 192.0.2.1') echo '[{"dev":"imt342-mock","prefsrc":"192.0.2.2"}]' ;;
    '-j link show dev imt342-mock') echo '[{"link_type":"ether","flags":["UP","LOWER_UP"]}]' ;;
    '-j -4 address show dev imt342-mock') echo '[{"addr_info":[{"local":"192.0.2.2"}]}]' ;;
    *) exit 94 ;;
esac
""")
        self.stub("secret-tool", """
[[ "$*" == 'lookup application imt342-rws controller 192.0.2.1:80' ]] || exit 95
printf '%s\n' lookup >>"$TEST_TRACE"
[[ "$TEST_ACCOUNT" == available ]] || exit 1
printf 'usuario-prueba\tclave-prueba\n'
""")
        self.stub("podman", """
import os
from pathlib import Path
import sys

args = sys.argv[1:]
if args[0] == "info":
    print("true")
elif args[:2] in (["image", "exists"], ["network", "exists"]):
    pass
elif args[0] == "run":
    assert args[-4:] == ["python3", "/opt/imt342-demo/irc5_probe.py",
                         "probe", "/tmp/imt342-backends.yaml"]
    assert sys.stdin.buffer.read() == b"usuario-prueba\\0clave-prueba\\0"
    with Path(os.environ["TEST_TRACE"]).open("a") as trace:
        print("probe", file=trace)
    print("Sonda RWS simulada")
else:
    sys.exit(96)
""", interpreter=sys.executable)

    def test_saved_account_reaches_probe_without_echo(self):
        self.stub_account_dependencies()
        result = self.run_script("irc5_demo.sh", "diagnose", "--config", self.config)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Sonda RWS simulada", result.stdout)
        self.assertEqual(self.trace.read_text().splitlines(), ["lookup", "probe"])
        self.assertNotIn("usuario-prueba", result.stdout + result.stderr)
        self.assertNotIn("clave-prueba", result.stdout + result.stderr)

    def test_missing_account_does_not_call_probe(self):
        self.stub_account_dependencies()
        self.env["TEST_ACCOUNT"] = "missing"
        result = self.run_script("irc5_demo.sh", "diagnose", "--config", self.config)
        self.assertEqual(result.returncode, 1)
        self.assertIn("Llavero RWS bloqueado o sin cuenta", result.stderr)
        self.assertEqual(self.trace.read_text().splitlines(), ["lookup"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
