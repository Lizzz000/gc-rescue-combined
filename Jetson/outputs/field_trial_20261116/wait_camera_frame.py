"""Bounded, read-only first-image check for a newly started vision process."""
import time
import urllib.error
import urllib.request

deadline = time.monotonic() + 15.0
last_error = 'no image'
while time.monotonic() < deadline:
    try:
        with urllib.request.urlopen('http://127.0.0.1:8080/snapshot.jpg', timeout=1.0) as response:
            frame = response.read(16 * 1024 * 1024)
            if response.status == 200 and len(frame) > 4 and frame[:2] == b'\xff\xd8' and frame[-2:] == b'\xff\xd9':
                print('Camera stream has produced a complete JPEG')
                break
            last_error = 'snapshot was not a complete JPEG'
    except (OSError, urllib.error.URLError) as error:
        last_error = str(error)
    time.sleep(0.1)
else:
    raise SystemExit('No camera frame within startup deadline: ' + last_error)
