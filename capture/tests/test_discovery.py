from insight_capture.discovery import TopicInfo as T, discover

IMG, CI, IMU = "sensor_msgs/msg/Image", "sensor_msgs/msg/CameraInfo", "sensor_msgs/msg/Imu"
ODOM, TF = "nav_msgs/msg/Odometry", "tf2_msgs/msg/TFMessage"
COMP = "sensor_msgs/msg/CompressedImage"


def cam(prefix, depth=False, pubs=1):
    t = [T(f"{prefix}/rgb/image_raw", IMG, pubs), T(f"{prefix}/rgb/camera_info", CI, pubs),
         T(f"{prefix}/left/image_raw", IMG, pubs), T(f"{prefix}/left/camera_info", CI, pubs),
         T(f"{prefix}/right/image_raw", IMG, pubs), T(f"{prefix}/right/camera_info", CI, pubs),
         T(f"{prefix}/imu", IMU, pubs), T(f"{prefix}/vio", ODOM, pubs),
         T("/tf", TF, 1), T("/tf_static", TF, 1)]
    if depth:
        t += [T(f"{prefix}/depth/image_raw", IMG, pubs), T(f"{prefix}/depth/camera_info", CI, pubs)]
    return t


def test_any_prefix(profile):
    for pfx in ("/insight9", "/camera/insight", "/a/b/c"):
        d = discover(cam(pfx), profile)
        assert d.found and d.prefix == pfx
        assert d.missing_required == []
        assert "depth" in d.missing_optional or d.matches["depth"] == []


def test_depth_off_vs_on(profile):
    d = discover(cam("/i"), profile)
    assert not d.depth_topic_present and not d.depth_publishing
    d = discover(cam("/i", depth=True, pubs=1), profile)
    assert d.depth_publishing
    d = discover(cam("/i", depth=True, pubs=0)[:-2] + [T("/i/depth/image_raw", IMG, 0)], profile)
    assert d.depth_topic_present and not d.depth_publishing


def test_foreign_device_not_mistaken(profile):
    other = [T("/odin1/image/compressed", COMP, 1), T("/odom", ODOM, 1), T("/cmd_vel", "geometry_msgs/msg/Twist", 1)]
    d = discover(other, profile)
    assert not d.found


def test_picks_best_prefix_among_two(profile):
    topics = cam("/insight9") + [T("/other/imu", IMU, 1)]
    d = discover(topics, profile)
    assert d.prefix == "/insight9"
    assert d.candidates["/insight9"] > d.candidates.get("/other", 0)


def test_explicit_prefix(profile):
    d = discover(cam("/x") + cam("/y"), profile, prefix="/y")
    assert d.prefix == "/y" and all(m.topic.startswith("/y") for m in d.matches["rgb"])


def test_missing_required_reported(profile):
    t = [x for x in cam("/i") if not x.name.endswith("/vio")]
    d = discover(t, profile)
    assert d.found and "vio" in d.missing_required


def test_raw_vs_compressed_choice(profile):
    t = cam("/i") + [T("/i/rgb/image_raw/compressed", COMP, 1)]
    d = discover(t, profile)
    assert [m.topic for m in d.chosen("raw")["rgb"]] == ["/i/rgb/image_raw"]
    assert [m.topic for m in d.chosen("compressed")["rgb"]] == ["/i/rgb/image_raw/compressed"]
    assert len(d.chosen("both")["rgb"]) == 2


def test_extras(profile):
    t = cam("/i") + [T("/i/diagnostics", "diagnostic_msgs/msg/DiagnosticArray", 1), T("/i/foo", "std_msgs/msg/String", 1)]
    d = discover(t, profile)
    assert [m.topic for m in d.extras] == ["/i/diagnostics"]
