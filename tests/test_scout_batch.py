from types import SimpleNamespace

from src.scout_batch import build_views, make_overviews


def point(x, y):
    return SimpleNamespace(location=SimpleNamespace(x=x, y=y, z=2))


def test_spread_and_pose_count():
    views = build_views([point(0, 0), point(1, 0), point(100, 0)], 2, [30, 60], -45)
    assert len(views) == 16
    assert {v["spawn_index"] for v in views} == {0, 2}
    assert {v["pose"][2] for v in views} == {32, 62}
    assert {v["pose"][4] for v in views} == {0, 90, 180, 270}


def test_no_spawn_is_error():
    import pytest

    with pytest.raises(ValueError, match="spawn"):
        build_views([], 12, [40], -45)


def test_overview_paginates(tmp_path):
    from PIL import Image

    for i in range(25):
        Image.new("RGB", (64, 36), "red").save(tmp_path / f"{i:04d}.png")
    paths = make_overviews(tmp_path)
    assert len(paths) == 2
    assert all(p.exists() for p in paths)
    assert len(make_overviews(tmp_path)) == 2
