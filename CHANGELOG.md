# CHANGELOG


## v0.4.0 (2025-05-19)

### Features

- Plot CA topics instead of actuator_controls
  ([`4131ea7`](https://github.com/aviant-tech/flight_review/commit/4131ea7c3f99c17b7cd9295430158565c8c595b5))

Still plot actuator_controls without CA


## v0.3.0 (2025-05-19)

### Features

- Plot all actuator_outputs instance
  ([`9804997`](https://github.com/aviant-tech/flight_review/commit/98049976d6632e5358c06744f1c6a85d08462d3d))

Was missing AUX PWM and AUX DSHOT


## v0.2.1 (2025-05-07)

### Build System

- Staging image is staging-latest, not latest
  ([`47126ac`](https://github.com/aviant-tech/flight_review/commit/47126ac28c70d83925608cb66e70c33e61b4c51b))


## v0.2.0 (2025-05-07)

### Features

- Plot actuator_outputs instances 0, 1, and 2
  ([`f695efa`](https://github.com/aviant-tech/flight_review/commit/f695efa4c83b50404133a33d29d2afc3fb79db50))


## v0.1.2 (2025-05-07)

### Build System

- Fix compose file location in deploy jobs
  ([`f9ed709`](https://github.com/aviant-tech/flight_review/commit/f9ed7091ad3ac4c377e92dcb2b43f07a571c709c))


## v0.1.1 (2025-05-07)

### Build System

- Fix dockerfile location in deploy job
  ([`e0f371d`](https://github.com/aviant-tech/flight_review/commit/e0f371d1e69cd66e6915201cc84f3e4bbd376656))


## v0.1.0 (2025-05-07)

### Bug Fixes

- Add use_proxy and domain
  ([`d978770`](https://github.com/aviant-tech/flight_review/commit/d97877091b6c07d10550e7c2ff1c5c0177fb2f2c))

- Avoid unnecessary indentation
  ([`fc26422`](https://github.com/aviant-tech/flight_review/commit/fc2642243e149b0e25ab0108d8860f1f5b52bce9))

- Jupyter does not need to be in requirements.txt
  ([`b361a5f`](https://github.com/aviant-tech/flight_review/commit/b361a5fb00e4ec80d193fd3c1e028a8b78e3530b))

- Typo in variance topic for distance sensor plot
  ([`a895b32`](https://github.com/aviant-tech/flight_review/commit/a895b328600a79e9f1f768b4a9e0ae820b6dc999))

- Update Jinja version to 3.0 in requirements.txt
  ([`15805fd`](https://github.com/aviant-tech/flight_review/commit/15805fd71719f9e624dec90d339fbd4f01b8e225))

Requirements install failed for me with Jinja 2

### Build System

- Ci/cd pipelie to aws
  ([`6e21ede`](https://github.com/aviant-tech/flight_review/commit/6e21ede89f7fa17a8a3194b65eb9d6215bed18ad))

- Deploy with docker compose
  ([`c0b07e6`](https://github.com/aviant-tech/flight_review/commit/c0b07e65c8fe85c682543791b5a18702e133e237))

Create docker compose config with shared volume where pyulog.sqlite is stored, to share file with
  fms

- Filght -> flight in docker tag
  ([`d03643d`](https://github.com/aviant-tech/flight_review/commit/d03643d704440f02b0a23e6d772cc03fed8aaa2d))

- Main branch is called master
  ([`c61f87a`](https://github.com/aviant-tech/flight_review/commit/c61f87a18660d016f723f9f7fcff42b96aa84bcc))

- Remove nonexisting fftw3 install
  ([`607a793`](https://github.com/aviant-tech/flight_review/commit/607a7934821fcedf4d5209f4b9b724779654604d))

- Remove python3.6 from test matrix
  ([`f101d00`](https://github.com/aviant-tech/flight_review/commit/f101d00c4cccc08d548a6924f5ef0d1300aa1c20))

### Continuous Integration

- Enable pylint add helpful messages
  ([`a9344f8`](https://github.com/aviant-tech/flight_review/commit/a9344f8ae7ea72b3e879f66812562c5c1ff3bc11))

- Fix lock check status
  ([`ab5a057`](https://github.com/aviant-tech/flight_review/commit/ab5a057b2301e73f60b863ab9a88cdc2bd495dc9))

- Switch from travis to github actions
  ([`88e901b`](https://github.com/aviant-tech/flight_review/commit/88e901b70b6b786172344c4d59e24a83203a145f))

### Documentation

- Install commands and heading upgrades
  ([`373e442`](https://github.com/aviant-tech/flight_review/commit/373e4423a8b9f2401e86d0010851c1c3eac50cf4))

The previous headings where getting out of hand when adding sub headings, I also moved the build
  status badge below the main title.

- Pipenv shell
  ([`14f9a86`](https://github.com/aviant-tech/flight_review/commit/14f9a868aab5ed1a9aedcc56deadacbd9ad03cf5))

### Features

- Fix vibration level plot
  ([`2fe2515`](https://github.com/aviant-tech/flight_review/commit/2fe25152893f741e9a6dcaa4d1a3365879ae1740))

Uses sensor_imu_status instead of estimator_status

- Log instead of print
  ([`e46c5b8`](https://github.com/aviant-tech/flight_review/commit/e46c5b859808c8a7d2722a73b544c483b853283c))

### Refactoring

- Move all html templates to plot_app/templates
  ([`9ddd19c`](https://github.com/aviant-tech/flight_review/commit/9ddd19c8dac124f3d98ab96d476ec7043bc336c7))

this also fixes the main.js include for the browse & upload page

- Move flight mode changes into generate_plots()
  ([`239df40`](https://github.com/aviant-tech/flight_review/commit/239df405b59be922fd96d3826bfb7f4ad15c49a0))

- Move flight modes into separate helper method
  ([`a890c9b`](https://github.com/aviant-tech/flight_review/commit/a890c9b8e65de1d5a83f9879b2eb3dd63e9f1fb5))

- Move flight_modes_table & airframe parsing into helper
  ([`9648eb3`](https://github.com/aviant-tech/flight_review/commit/9648eb303fe1ddaba05652acd10fc6ba0e593002))

- Move main.js to templates and include it via jinja
  ([`a1bc031`](https://github.com/aviant-tech/flight_review/commit/a1bc03161f4432a518d2ede893beb10b4c902279))

This allows to pass jinja arguments to the JS file.

- Rename plot_height['gps_map'] to 'large' to be more generic
  ([`43b8767`](https://github.com/aviant-tech/flight_review/commit/43b87678cca7f8e821426c6a745d6dd7198c6e14))
