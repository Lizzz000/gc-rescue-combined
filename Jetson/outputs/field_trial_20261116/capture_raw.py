"""Save one original processed BGR frame, before the stream draws overlays."""
import json
from pathlib import Path
import time
import cv2
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

out = Path('outputs/field_trial_20261116/raw_01')
out.mkdir(parents=True, exist_ok=True)
rclpy.init()
node = rclpy.create_node('preflight_capture_field_input')
seen = []
def receive(message):
    if seen:
        return
    if message.encoding != 'bgr8' or len(message.data) != message.step * message.height:
        raise RuntimeError('Expected complete BGR8 image')
    pixels = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    image = pixels[:, :message.width * 3].reshape(message.height, message.width, 3)
    if not cv2.imwrite(str(out / 'live_input.png'), image):
        raise RuntimeError('PNG write failed')
    seen.append({'width': message.width, 'height': message.height, 'step': message.step,
                 'stamp': [message.header.stamp.sec, message.header.stamp.nanosec], 'encoding': message.encoding})
node.create_subscription(Image, '/detect_node/image_processed', receive,
                         QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
deadline = time.monotonic() + 10
while not seen and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=.2)
node.destroy_node()
rclpy.shutdown()
if not seen:
    raise SystemExit('No processed frame received')
(out / 'live_input.json').write_text(json.dumps(seen[0], indent=2) + '\n')
print(json.dumps(seen[0]))
