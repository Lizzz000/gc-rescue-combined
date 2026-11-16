"""Read-only capture of an unannotated image and field geometry with identical source stamp."""
import argparse
import json
from pathlib import Path
import time
import cv2
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from interface_pkg.msg import FieldObservation
from vision_geometry_probe import Collector, stamp_ns

parser = argparse.ArgumentParser()
parser.add_argument('--name', required=True)
args = parser.parse_args()
out = Path('outputs/field_trial_20261116') / args.name
out.mkdir(parents=True, exist_ok=True)
images, fields, done = {}, {}, []
collector = Collector('field')
collector.begin(15)
rclpy.init()
node = rclpy.create_node('field_capture_matched')

def save(key):
    if done or key not in images or key not in fields:
        return
    msg, row = images[key], fields[key]
    if msg.encoding != 'bgr8' or len(msg.data) != msg.step * msg.height:
        raise ValueError('Expected complete BGR8 image')
    pixels = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    pixels = pixels[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    if not cv2.imwrite(str(out / 'raw.png'), pixels):
        raise RuntimeError('PNG write failed')
    record = {'width': msg.width, 'height': msg.height, 'source_stamp_ns': key,
              'matched_source_stamp': key == row['source_stamp_ns'], 'field': row}
    (out / 'geometry.json').write_text(json.dumps(record, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    done.append(record)

def trim(cache):
    while len(cache) > 8:
        del cache[next(iter(cache))]

def image_callback(msg):
    key = stamp_ns(msg)
    images[key] = msg
    trim(images)
    save(key)

def field_callback(msg):
    collector.field_callback(msg)
    row = collector.rows['field'][-1]
    collector.rows['field'].clear()
    key = row['source_stamp_ns']
    fields[key] = row
    trim(fields)
    save(key)

qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
node.create_subscription(Image, '/detect_node/image_processed', image_callback, qos)
node.create_subscription(FieldObservation, '/detect_node/field/results', field_callback, qos)
deadline = time.monotonic() + 10
while not done and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=.1)
node.destroy_node()
rclpy.shutdown()
if not done:
    raise SystemExit('No image/field pair with identical source stamp')
print(json.dumps({'path': str(out), 'source_stamp_ns': done[0]['source_stamp_ns'],
                  'valid': done[0]['field']['valid'], 'matched': True}))
