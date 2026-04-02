from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ccoli import cli


def test_config_integration_set_and_test(tmp_path, monkeypatch):
    server_dir = tmp_path / "server"
    server_dir.mkdir()
    (server_dir / "config.yaml").write_text("", encoding="utf-8")

    monkeypatch.setattr(cli, "_repo_root", lambda: tmp_path)

    rc = cli.main(["config", "integration", "set", "weather", "--api-key", "secret-key"])
    assert rc == 0

    rc = cli.main(["config", "integration", "test", "weather"])
    assert rc == 0


def test_config_voice_threshold_validation(tmp_path, monkeypatch):
    server_dir = tmp_path / "server"
    server_dir.mkdir()
    (server_dir / "config.yaml").write_text("", encoding="utf-8")
    monkeypatch.setattr(cli, "_repo_root", lambda: tmp_path)

    rc = cli.main(["config", "voice-id", "threshold", "--value", "1.2"])
    assert rc == 2

    rc = cli.main(["config", "voice-id", "threshold", "--value", "0.75"])
    assert rc == 0


def test_config_voice_delete_removes_profile(tmp_path, monkeypatch):
    server_dir = tmp_path / "server"
    server_dir.mkdir()
    (server_dir / "config.yaml").write_text("", encoding="utf-8")
    voice_dir = server_dir / "data" / "voice_profiles"
    voice_dir.mkdir(parents=True)
    (voice_dir / "profiles.json").write_text('{"alice": {"user": "alice"}}', encoding="utf-8")
    (voice_dir / "alice.npy").write_bytes(b"dummy")
    monkeypatch.setattr(cli, "_repo_root", lambda: tmp_path)

    rc = cli.main(["config", "voice-id", "delete", "--user", "alice"])
    assert rc == 0
    assert not (voice_dir / "alice.npy").exists()
    assert "alice" not in (voice_dir / "profiles.json").read_text(encoding="utf-8")


def test_config_wifi_mode_wired(tmp_path, monkeypatch):
    server_dir = tmp_path / "server"
    server_dir.mkdir()
    (server_dir / "config.yaml").write_text("", encoding="utf-8")

    monkeypatch.setattr(cli, "_repo_root", lambda: tmp_path)

    rc = cli.main(["config", "wifi", "OfficeLAN", "password", "dummy", "port", "5009", "mode", "wired"])
    assert rc == 0

    config_text = (server_dir / "config.yaml").read_text(encoding="utf-8")
    secrets_text = (tmp_path / "arduino" / "atom_echo_m5stack_esp32_ino" / "device_secrets.h").read_text(encoding="utf-8")
    assert "mode: wired" in config_text
    assert 'CONNECTION_MODE = "wired"' in secrets_text


def test_setup_writes_wifi_connection_settings(tmp_path, monkeypatch):
    server_dir = tmp_path / "server"
    server_dir.mkdir()
    (server_dir / "config.yaml").write_text("", encoding="utf-8")

    monkeypatch.setattr(cli, "_repo_root", lambda: tmp_path)

    rc = cli.main(
        [
            "setup",
            "--install-target",
            "api",
            "--provider",
            "gemini",
            "--stt-device",
            "cpu",
            "--connection-mode",
            "wifi",
            "--wifi-ssid",
            "OfficeLAN",
            "--wifi-password",
            "secret-pass",
            "--server-ip",
            "192.168.0.20",
            "--server-port",
            "5009",
            "--skip-install",
            "--yes",
        ]
    )
    assert rc == 0

    config_text = (server_dir / "config.yaml").read_text(encoding="utf-8")
    secrets_text = (tmp_path / "arduino" / "atom_echo_m5stack_esp32_ino" / "device_secrets.h").read_text(encoding="utf-8")
    assert "mode: wifi" in config_text
    assert "port: 5009" in config_text
    assert 'CONNECTION_MODE = "wifi"' in secrets_text
    assert 'SSID = "OfficeLAN"' in secrets_text
    assert 'PASS = "secret-pass"' in secrets_text
    assert 'SERVER_IP = "192.168.0.20"' in secrets_text
