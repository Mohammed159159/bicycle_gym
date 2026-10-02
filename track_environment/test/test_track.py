from track_environment.track import Track, DEFAULT_TRACK_FILE


def test_track_loading():
    """Verify that the default racetrack loads waypoints and has positive perimeter."""
    track = Track(track_file=DEFAULT_TRACK_FILE)
    assert len(track.waypoints) > 100
    assert track.total_length > 10.0
    x0, y0, psi0 = track.start_pose
    assert isinstance(x0, float)
    assert isinstance(y0, float)
    assert isinstance(psi0, float)
