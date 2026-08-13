Robot Simulator - WebAssembly build
===================================

Contents
--------
  index.html            Entry point. Open this one.
  RobotSimulator.js     Emscripten loader and JS glue.
  RobotSimulator.wasm   The compiled application.
  RobotSimulator.data   Preloaded filesystem: built-in asset library, default station, and UI font.
  THIRD-PARTY.txt       Licences for the bundled third-party components.

Running it
----------
WebAssembly cannot be loaded from a file:// URL - the browser blocks it as a
cross-origin request - so the folder has to be served over HTTP. Any static
server will do, for example from inside the unzipped folder:

    python -m http.server 8000

then open http://localhost:8000/ in a browser.

Every package in library/packages is baked into RobotSimulator.data under /packages.
The FAIRINO FR20 + Gudel TMF-1 machine-tending station loads automatically from
/stations/fairino_gudel_machine_tending.station.json, so there is nothing to open by hand.

Requirements
------------
A browser with WebGL 2 and WebAssembly SIMD support. Current Chrome, Edge and
Firefox releases qualify.

Differences from the desktop build
----------------------------------
- PhysX 5.9 is compiled into the WebAssembly module as a CPU-only static backend.
  Conveyor contact target velocities, rigid workpieces, gripper forces, gravity,
  floor collisions and deletion volumes use the same PhysX implementation as the
  desktop build. It runs with a zero-worker dispatcher, so serving the files does
  not require SharedArrayBuffer or COOP/COEP response headers.
- Hardware IO works, through the Web Serial API. Because a browser only reveals a
  device the user has explicitly granted, the tab has a "Request port..." button
  where the desktop build simply lists the COM ports; once a device is allowed it
  appears in the dropdown and Connect behaves as it does on the desktop.

  Web Serial is Chromium only - Chrome or Edge, not Firefox or Safari - and needs
  a secure context, so serve over HTTPS or from localhost.

- The blocking --hardware-io-* command line tools are not built for the web,
  since there is no command line in a browser tab.
- The UI font is DroidSans rather than Segoe UI, which is not redistributable.
- Physics and Simulate run on the frame thread. The desktop build does the trajectory build
  on a background thread and streams partial results into the graphs; in the
  browser the tab stops responding until the build finishes. Short programs are
  imperceptible, long ones are not.
- Open... and Load use the browser's file picker, which is the ordinary OS file
  dialog. The chosen file is copied into the app's in-memory filesystem, so it is
  not kept once the tab is closed.
- Save uses the File System Access API where it exists, which is a real Save As
  dialog; on Firefox and Safari it falls back to downloading the file.
