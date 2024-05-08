# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0] - 2023-11-23

### Added

- Add by default Teleop plugin in rviz.
- Add a basic node to generate AckermannDrive command from Teleop message. You can now move the vehicle from Teleop plugin.
- Add a dirty highbay world (mainly for debug test).

### Changed

- Update IMU frequency (20 to 100 Hz)
- Update lidar VLP frequency (10 to 20 Hz)

### Fixed

- Update the command to launch highbay_track.world without issue in the readme.md
- Restore previous gem_control which work with simple_track_green.world and highbay_track.world

