from pathlib import Path


STATIC_DIR = Path(__file__).resolve().parents[1] / "web" / "static"


def test_dashboard_shell_defaults_to_english_and_loads_split_assets():
    html = (STATIC_DIR / "index.html").read_text(encoding="utf-8")

    assert '<html lang="en">' in html
    assert 'href="/dashboard.css"' in html
    assert 'src="/dashboard.js"' in html
    # Greenhouse type system is loaded from Google Fonts.
    assert "fonts.googleapis.com" in html
    assert "Fraunces" in html
    # Core JS hooks the redesign must preserve.
    assert 'id="locale-select"' in html
    assert 'data-tab="diagnostics"' in html
    assert 'id="runtime-summary-cards"' in html
    assert 'id="config-json"' in html
    assert 'id="hero-headline"' in html
    # Accessibility: skip-link to the main landmark.
    assert 'class="skip-link"' in html
    assert 'id="main"' in html
    # Mascot uses the SVG asset (favicon + bar logo + hero); the old PNG is gone.
    assert html.count("/brand-assets/ccoli.svg") == 3
    assert "/brand-assets/ccoli.png" not in html


def test_dashboard_css_uses_greenhouse_system_and_text_safe_rules():
    css = (STATIC_DIR / "dashboard.css").read_text(encoding="utf-8")

    # Greenhouse tokens: one warm carrot accent + distinctive type.
    assert "--carrot" in css
    assert "Fraunces" in css
    assert "Space Mono" in css
    # Working dark theme block (the previously-dead [data-theme="dark"] is authored).
    assert '[data-theme="dark"]' in css
    # Accessibility upgrades that ship regardless of aesthetic.
    assert ":focus-visible" in css
    assert "prefers-reduced-motion" in css
    # Text-safe layout + the live status / hero signatures.
    assert "overflow-wrap:anywhere" in css
    assert ".hero-figure" in css
    assert ".dot.live" in css


def test_dashboard_js_keeps_contract_and_fixes_dark_mode():
    js = (STATIC_DIR / "dashboard.js").read_text(encoding="utf-8")

    assert 'localStorage.getItem("ccoli.locale") || "en"' in js
    assert '"/api/diagnostics/"' in js
    # Dark-mode fix: dark now sets the attribute the CSS keys off of.
    assert 'setAttribute("data-theme", "dark")' in js
