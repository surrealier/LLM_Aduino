from pathlib import Path


def test_dashboard_minimizes_copy_with_css_rules():
    html = (Path(__file__).resolve().parents[1] / "web" / "static" / "index.html").read_text(encoding="utf-8")

    assert ".hero-grid>.panel{display:none}" in html
    assert ".brand .muted,.top .muted,.hero-copy>div p:not(.eyebrow),.overview .hint,.status .meta,.nav button small,.metrics .row span:first-child{display:none}" in html
    assert 'id="hero-headline"' in html
    assert 'id="overview-schedules"' in html
    assert 'id="chat-feed"' in html
    assert 'id="config-json"' in html
    assert 'id="logs-view"' in html
