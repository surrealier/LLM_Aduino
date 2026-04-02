from pathlib import Path


STATIC_DIR = Path(__file__).resolve().parents[1] / "web" / "static"


def test_dashboard_shell_defaults_to_english_and_uses_split_assets():
    html = (STATIC_DIR / "index.html").read_text(encoding="utf-8")

    assert '<html lang="en">' in html
    assert 'href="/dashboard.css"' in html
    assert 'src="/dashboard.js"' in html
    assert 'id="locale-select"' in html
    assert 'data-tab="diagnostics"' in html
    assert 'id="runtime-summary-cards"' in html
    assert 'id="config-json"' in html
    assert 'id="hero-headline"' in html
    assert html.count('/brand-assets/ccoli.png') == 1


def test_dashboard_css_preserves_text_safe_layout_rules():
    css = (STATIC_DIR / "dashboard.css").read_text(encoding="utf-8")
    js = (STATIC_DIR / "dashboard.js").read_text(encoding="utf-8")

    assert ".title-clamp{display:-webkit-box;-webkit-line-clamp:2;-webkit-box-orient:vertical;overflow:hidden" in css
    assert ".body-copy{font-size:.82rem;line-height:1.5;color:var(--muted)}" in css
    assert "overflow-wrap:anywhere" in css
    assert ".hero-figure" in css
    assert 'localStorage.getItem("ccoli.locale") || "en"' in js
    assert '"/api/diagnostics/"' in js
