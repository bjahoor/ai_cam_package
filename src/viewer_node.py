#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage

# ---- AI helper (topic-driven OWL-ViT; defaults to "apple") ----
from std_msgs.msg import String
import os
import cv2
from typing import List

_AI = {
    "pipe": None,
    "labels": ["apple"],     # default detection labels
    "thr": 0.1,             # default threshold
    "inited": False,
    "notified_empty": False,
}

def ai_update_labels(label_string: str, node=None):
    """
    Update detection labels from a CSV string, e.g. "apple, red apple, fruit".
    Resets the 'no detections' notice so you get one info log again if empty.
    """
    try:
        labels = [s.strip() for s in label_string.split(",") if s.strip()]
        if labels:
            _AI["labels"] = labels
            _AI["notified_empty"] = False
            if node:
                node.get_logger().info(f"AI labels set to: {labels}")
    except Exception as e:
        if node:
            node.get_logger().warn(f"AI label update failed: {e}")

def ai_detect(frame_bgr, node):
    """
    Minimal, CPU-only zero-shot detection via transformers.pipeline.
    - Default labels: ['apple']
    - Override at process start with env:
        AI_PROMPT="apple,banana"  AI_THR=0.10
    - Live updates: call ai_update_labels("comma,separated,labels") from a ROS topic.
    Safe no-op on any error.
    """
    try:
        from PIL import Image
        from transformers import pipeline
    except Exception as e:
        try:
            node.get_logger().warn(f"AI detect disabled (imports failed): {e}")
        except Exception:
            pass
        return frame_bgr

    try:
        # One-time init: env overrides for startup defaults
        if not _AI["inited"]:
            env_labels = os.getenv("AI_PROMPT")
            if env_labels:
                ai_update_labels(env_labels, node=node)
            try:
                _AI["thr"] = float(os.getenv("AI_THR", _AI["thr"]))
            except Exception:
                pass
            _AI["inited"] = True

        # Lazy model load (CPU)
        if _AI["pipe"] is None:
            _AI["pipe"] = pipeline(
                task="zero-shot-object-detection",
                model="google/owlvit-base-patch32",
                device=-1,  # CPU
            )
            if node:
                node.get_logger().info("AI pipeline ready (OWL-ViT, CPU).")

        # Prepare image and run
        img_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(img_rgb)
        out = _AI["pipe"](pil_img, candidate_labels=_AI["labels"])

        if not out and not _AI["notified_empty"]:
            if node:
                node.get_logger().info(
                    f"AI: no detections for labels={_AI['labels']} (try lower AI_THR)"
                )
            _AI["notified_empty"] = True

        # Draw boxes
        thr = float(_AI["thr"])
        for det in out:
            try:
                score = float(det.get("score", 0.0))
                if score < thr:
                    continue
                box = det.get("box", {})
                x1, y1 = int(box["xmin"]), int(box["ymin"])
                x2, y2 = int(box["xmax"]), int(box["ymax"])
                label = det.get("label", _AI["labels"][0] if _AI["labels"] else "obj")
                cv2.rectangle(frame_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    frame_bgr,
                    f"{label}:{score:.2f}",
                    (x1, max(0, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA,
                )
            except Exception:
                continue

    except Exception as e:
        try:
            node.get_logger().warn(f"AI detect runtime error (skipping): {e}")
        except Exception:
            pass

    return frame_bgr
# ---- end helper ----






class MinimalViewer(Node):
    def __init__(self):
        super().__init__('minimal_viewer')
        self.create_subscription(CompressedImage, '/camera/camera/color/image_raw/compressed', self.color_cb, 10)
        self.create_subscription(CompressedImage, '/camera/camera/depth/image_rect_raw/compressedDepth', self.depth_cb, 10)
        self.create_subscription(String, "/ai/prompt", self._prompt_cb, 10)


    def color_cb(self, msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), 1)   # decode
        frame = ai_detect(frame, self)  # uses current labels; default ["apple"]
        cv2.imshow("Color", frame)
        cv2.waitKey(1)

    def depth_cb(self, msg):
        #alpha scaling and colormap can be modified!!!
        cv2.imshow("Depth", cv2.applyColorMap(cv2.convertScaleAbs(cv2.imdecode(np.frombuffer(msg.data[12:], np.uint8), -1), alpha=0.1), cv2.COLORMAP_JET))
        cv2.waitKey(1)

    def _prompt_cb(self, msg: String):
        ai_update_labels(msg.data, self)  # updates helper’s labels


def main():
    rclpy.init()
    rclpy.spin(MinimalViewer())
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()