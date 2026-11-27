"""Insight 9 stereo-first mapping pipeline (phase 1).

Works straight from a recorded bag (MCAP or SQLite3, a Session directory or a bare bag), driven by the real
fw 2.1.5 topic set documented in capture/DATA_CONTRACT.md v0.3. Stereo (infra1/infra2 + CameraInfo) is the only
geometric input; IMU / Depth / VIO are indexed and kept for later fusion but are NOT used (semantics unverified).
"""
