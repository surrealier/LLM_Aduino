"""Tests for the interactive setup/install flow helpers."""
import os
import sys

import yaml


sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from ccoli.cli import (  # noqa: E402
    DEFAULT_LLM_MODELS,
    _apply_setup_choice,
    _build_setup_choice,
    _default_stt_device,
    build_parser,
)


def test_parser_has_setup_command():
    parser = build_parser()
    choices = parser._subparsers._group_actions[0].choices
    assert "setup" in choices


def test_default_stt_device_uses_cpu_on_mac(monkeypatch):
    monkeypatch.setattr("ccoli.cli.sys.platform", "darwin")
    assert _default_stt_device() == "cpu"


def test_build_setup_choice_for_ollama():
    choice = _build_setup_choice(
        install_target="ollama",
        provider=None,
        model=None,
        api_key=None,
        stt_device="cpu",
    )

    assert choice.install_target == "ollama"
    assert choice.provider == "ollama"
    assert choice.model == DEFAULT_LLM_MODELS["ollama"]
    assert choice.install_extras == ("runtime",)
    assert choice.api_key is None


def test_build_setup_choice_for_gemini_api():
    choice = _build_setup_choice(
        install_target="api",
        provider="gemini",
        model=None,
        api_key="gemini-secret",
        stt_device="cpu",
    )

    assert choice.install_target == "api"
    assert choice.provider == "gemini"
    assert choice.model == DEFAULT_LLM_MODELS["gemini"]
    assert choice.install_extras == ("runtime",)
    assert choice.api_key == "gemini-secret"


def test_apply_setup_choice_updates_config_and_env(tmp_path):
    root = tmp_path
    server_dir = root / "server"
    server_dir.mkdir(parents=True)
    (server_dir / "config.yaml").write_text("server:\n  port: 5001\n", encoding="utf-8")

    choice = _build_setup_choice(
        install_target="api",
        provider="gemini",
        model=None,
        api_key="gemini-secret",
        stt_device="cpu",
    )

    config_path, env_path = _apply_setup_choice(root, choice)

    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    assert config["setup"]["install_target"] == "api"
    assert config["stt"]["device"] == "cpu"
    assert config["llm"]["provider"] == "gemini"
    assert config["llm"]["model"] == DEFAULT_LLM_MODELS["gemini"]
    assert config["llm"]["base_url"] == ""
    assert env_path.read_text(encoding="utf-8").strip() == "GEMINI_API_KEY=gemini-secret"
