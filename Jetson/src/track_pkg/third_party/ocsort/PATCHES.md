# Local OC-SORT changes

The upstream MIT license remains in `LICENSE`.

2026-11-18:

- On consecutive matched frames, store the new observation once. Occlusion
  replay still receives the complete history through the returning observation.

2026-11-15:

- Return `OCSortTrack` records with fixed-size float box geometry, integer class,
  float confidence and an independent `uint64_t` identity. Keep the process-wide
  identity counter integer throughout. This fixes duplicate IDs above 2^24 and
  signed IDs at the 32-bit boundary without changing association or box filtering.
- Remove the unused `KalmanBoxTracker::history` buffer. It had no readers and
  caused a second box conversion plus a stored dynamic vector on every prediction.
  KalmanFilter observation history used for occlusion replay is retained.
- Register the core regression tests in the package's `BUILD_TESTING` path and
  the standalone `test/host` CMake project. New regressions cover identity boundaries,
  reordering, class isolation and the production value adapter.

Existing local behavior retained: independent instances share the atomic identity
source, empty frames age tracks, observation history is bounded to the required
velocity/gap window, collapsed predictions are discarded consistently, confidence
threshold equality is accepted, and unique IoU matches skip direction cost work.
