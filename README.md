# AI Camera Viewer

This project builds on my compressed OpenCV RealSense viewer package, which provides low‑bandwidth streaming and dynamic resolution selection for both RGB and depth:
[https://github.com/bjahoor/cam_package](https://github.com/bjahoor/cam_package)

This extended version adds minimal **AI detection** using:

**Model used:** `google/owlvit-base-patch32` (zero-shot object detection)

---

## What the Node Does

When running, the node:

* Subscribes to:

  * `/camera/camera/color/image_raw/compressed`
  * `/camera/camera/depth/image_rect_raw/compressedDepth`
  * `/ai/prompt` (String)
* Shows two OpenCV windows: **Color** and **Depth**

---

## Color Image (with AI)

1. Decode the compressed JPEG into an OpenCV frame.
2. Run:

   ```python
   frame = ai_detect(frame, self)
   ```
3. Inside `ai_detect()`:

   * Loads the OWL-ViT model once (lazy load)
   * Uses `_AI["labels"]` (default: `"apple"`)
   * Draws bounding boxes + scores above `_AI["thr"]`
   * Falls back safely if the model is unavailable
4. The annotated frame is shown in the **Color** window.

### Where the AI Model Is Actually Called

Detection happens in this line inside `ai_detect()`:

```python
out = _AI["pipe"](pil_img, candidate_labels=_AI["labels"])
```

This is the exact call that runs the OWL-ViT model on the image using the current labels.

## Depth Image (unchanged)

* Depth is decoded, scaled, color-mapped (JET), and displayed.
* This is unchanged from the original project.

---

## Updating AI Labels Live

Send a message to `/ai/prompt` to update the labels used by `ai_detect()`:

```bash
ros2 topic pub --once /ai/prompt std_msgs/String '{data: "bottle"}'
```

This sets the AI to look for the label `"bottle"`.

General example:

```bash
ros2 topic pub --once /ai/prompt std_msgs/String '{data: "apple, banana"}'
```

This updates `_AI["labels"]` without restarting the node.

---

## Optional Environment Variables

```bash
AI_PROMPT="cat,dog"   # sets initial labels
AI_THR=0.15           # sets confidence threshold
```

These are read once on the first call to `ai_detect()`.
