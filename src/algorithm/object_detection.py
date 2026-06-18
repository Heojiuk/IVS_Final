#!/usr/bin/env python3
"""
detect.py
---------
Raspberry Pi 5 + Hailo AI HAT + Camera Module 3
yolov11n.hef real-time object detection (car / stop_sign / left_turn_sign)

Usage:
    python detect.py
    python detect.py --hef path/to/yolov11n.hef

Quit: press q
"""

import argparse
import numpy as np
import cv2
from pathlib import Path

# ===== Settings =============================================================
HEF_PATH    = "yolov11n.hef"
CLASS_NAMES   = ["car", "left_turn_sign", "stop_sign"]
CONF_THRESH = 0.5
IMG_SIZE    = 640
CAMERA_W    = 1920
CAMERA_H    = 1080

# Class colors (BGR)
COLORS = {
    "car":            (0,   255, 0),
    "left_turn_sign": (255, 165, 0),
    "stop_sign":      (0,   0,   255),
}

# ===== Sign class index constants ===========================================
CLS_CAR       = 0
CLS_LEFT_TURN = 2
CLS_STOP      = 1


# ===== Color-based sign correction ==========================================
def _color_ratio(hsv_crop, lower, upper, sat_mask):
    """
    Return the fraction of *saturated* pixels that fall inside an HSV range.
    Using saturated pixels as the denominator avoids the sign's white/black
    areas diluting the ratio and prevents low-saturation backgrounds
    (white floor, black border) from washing out the true sign color.
    """
    color_mask = cv2.inRange(hsv_crop, np.array(lower), np.array(upper))
    denom = max(int(np.sum(sat_mask)), 1)
    return int(np.sum((color_mask > 0) & sat_mask)) / denom


def correct_sign_class(frame_rgb, x1, y1, x2, y2, cls, conf):
    """
    Re-classify a detected sign box using dominant HSV color.

    stop_sign      -> dominant RED  (H:0-15 or H:155-180, high saturation)
    left_turn_sign -> dominant BLUE (H:105-130, high saturation)

    Key design decisions
    --------------------
    1. Denominator = saturated pixels only (S > 80).
       Signs contain white text / black borders that are achromatic; including
       them makes every ratio tiny and unreliable.

    2. Blue range starts at H=105 (not 90/100).
       The environment has a teal/cyan tape on the floor (H~90-105) that was
       leaking into the blue mask and flipping stop_sign <-> left_turn_sign.
       Pure sign-blue sits at H=105-130 and is unambiguous.

    3. Red S-threshold = 80 (not 120).
       STOP signs can appear darker under indoor lighting; a stricter sat
       threshold drops too many valid red pixels.

    Returns (corrected_cls, corrected_conf, color_vote)
      color_vote: 'red' | 'blue' | 'unknown'
    """
    if cls == CLS_CAR:
        return cls, conf, "n/a"

    # Clamp crop to frame bounds
    h_frame, w_frame = frame_rgb.shape[:2]
    x1c = max(0, x1); y1c = max(0, y1)
    x2c = min(w_frame, x2); y2c = min(h_frame, y2)
    crop = frame_rgb[y1c:y2c, x1c:x2c]

    if crop.size == 0:
        return cls, conf, "unknown"

    hsv = cv2.cvtColor(crop, cv2.COLOR_RGB2HSV)

    # Saturated-pixel mask used as denominator
    sat_mask = hsv[:, :, 1] > 80  # S > 80: only chromatic pixels

    # RED (wraps around 0 deg): H 0-15 and H 155-180
    red_ratio = (
        _color_ratio(hsv, [0,   80, 60], [15,  255, 255], sat_mask) +
        _color_ratio(hsv, [155, 80, 60], [180, 255, 255], sat_mask)
    )

    # BLUE (pure sign blue, H 105-130): teal tape is H~90-105 -> excluded
    blue_ratio = _color_ratio(hsv, [105, 80, 60], [130, 255, 255], sat_mask)

    THRESHOLD = 0.15  # at least 15% of saturated pixels must be the color

    if red_ratio >= blue_ratio and red_ratio >= THRESHOLD:
        color_vote = "red"
        corrected_cls = CLS_STOP
    elif blue_ratio > red_ratio and blue_ratio >= THRESHOLD:
        color_vote = "blue"
        corrected_cls = CLS_LEFT_TURN
    else:
        color_vote = "unknown"
        corrected_cls = cls  # keep model prediction

    # Slightly reduce confidence when we override the model's choice
    corrected_conf = conf * 0.90 if corrected_cls != cls else conf

    return corrected_cls, corrected_conf, color_vote


def correct_all_signs(frame_rgb, detections):
    """
    Apply color correction to every sign detection, then re-run NMS
    so that duplicate boxes for the same sign are collapsed.

    Returns a new detections list with corrected classes.
    """
    corrected = []
    for (x1, y1, x2, y2, conf, cls) in detections:
        new_cls, new_conf, vote = correct_sign_class(
            frame_rgb, x1, y1, x2, y2, cls, conf
        )
        corrected.append((x1, y1, x2, y2, new_conf, new_cls))

    # --- Re-run NMS per class after correction ----------------------------
    if not corrected:
        return []

    boxes   = np.array([[x1, y1, x2, y2] for x1, y1, x2, y2, _, _ in corrected])
    scores  = np.array([c for _, _, _, _, c, _ in corrected])
    classes = np.array([cl for _, _, _, _, _, cl in corrected])

    keep = []
    iou_thresh = 0.45
    for cls_id in np.unique(classes):
        idx   = np.where(classes == cls_id)[0]
        b     = boxes[idx]
        s     = scores[idx]
        order = s.argsort()[::-1]
        while order.size > 0:
            i = order[0]
            keep.append(idx[i])
            xx1   = np.maximum(b[i, 0], b[order[1:], 0])
            yy1   = np.maximum(b[i, 1], b[order[1:], 1])
            xx2   = np.minimum(b[i, 2], b[order[1:], 2])
            yy2   = np.minimum(b[i, 3], b[order[1:], 3])
            w     = np.maximum(0, xx2 - xx1)
            h_    = np.maximum(0, yy2 - yy1)
            inter = w * h_
            area_i = (b[i, 2] - b[i, 0]) * (b[i, 3] - b[i, 1])
            area_o = (b[order[1:], 2] - b[order[1:], 0]) * (b[order[1:], 3] - b[order[1:], 1])
            iou    = inter / (area_i + area_o - inter + 1e-6)
            order  = order[np.where(iou <= iou_thresh)[0] + 1]

    # Also suppress cross-class overlapping sign boxes
    # (e.g. stop_sign and left_turn_sign overlapping > 0.5 IoU -> keep higher conf)
    final_keep = list(set(keep))
    sign_indices = [i for i in final_keep if corrected[i][5] in (CLS_STOP, CLS_LEFT_TURN)]
    non_sign     = [i for i in final_keep if corrected[i][5] == CLS_CAR]

    # Greedy cross-class NMS for signs
    sign_indices.sort(key=lambda i: corrected[i][4], reverse=True)
    accepted = []
    for i in sign_indices:
        b1 = boxes[i]
        suppressed = False
        for j in accepted:
            b2   = boxes[j]
            xx1  = max(b1[0], b2[0]); yy1 = max(b1[1], b2[1])
            xx2  = min(b1[2], b2[2]); yy2 = min(b1[3], b2[3])
            w    = max(0, xx2 - xx1); h_ = max(0, yy2 - yy1)
            inter= w * h_
            a1   = (b1[2]-b1[0])*(b1[3]-b1[1])
            a2   = (b2[2]-b2[0])*(b2[3]-b2[1])
            iou  = inter / (a1 + a2 - inter + 1e-6)
            if iou > 0.40:
                suppressed = True
                break
        if not suppressed:
            accepted.append(i)

    return [corrected[i] for i in sorted(non_sign + accepted)]


# ===== Preprocessing ========================================================
def preprocess(frame):
    img = cv2.resize(frame, (IMG_SIZE, IMG_SIZE))
    img = np.ascontiguousarray(img)
    return np.expand_dims(img, axis=0)


# ===== Postprocessing (model output -> raw detections + NMS) ================
def postprocess(outputs, orig_w, orig_h, conf_thresh=CONF_THRESH):
    detections = []
    for output in outputs.values():
        if isinstance(output, (list, tuple)):
            for class_idx, class_dets in enumerate(output[0]):
                if class_dets is None:
                    continue
                for det in class_dets:
                    if len(det) < 5:
                        continue
                    y1, x1, y2, x2, conf = det[0], det[1], det[2], det[3], det[4]
                    if conf < conf_thresh:
                        continue
                    x1 = int(x1 * orig_w); y1 = int(y1 * orig_h)
                    x2 = int(x2 * orig_w); y2 = int(y2 * orig_h)
                    detections.append((x1, y1, x2, y2, float(conf), class_idx))
        else:
            output = np.squeeze(np.array(output))
            if output.ndim == 1:
                output = output[np.newaxis, :]
            for det in output:
                if len(det) < 6:
                    continue
                y1, x1, y2, x2 = det[0], det[1], det[2], det[3]
                conf = float(det[4]); cls = int(det[5])
                if conf < conf_thresh:
                    continue
                detections.append((
                    int(x1 * orig_w), int(y1 * orig_h),
                    int(x2 * orig_w), int(y2 * orig_h),
                    conf, cls
                ))

    # NMS (per class, from model output)
    if not detections:
        return []

    boxes   = np.array([[x1, y1, x2, y2] for x1, y1, x2, y2, _, _ in detections])
    scores  = np.array([conf for _, _, _, _, conf, _ in detections])
    classes = np.array([cls  for _, _, _, _, _, cls  in detections])

    keep = []
    iou_thresh = 0.45
    for cls_id in np.unique(classes):
        idx   = np.where(classes == cls_id)[0]
        b     = boxes[idx]; s = scores[idx]
        order = s.argsort()[::-1]
        while order.size > 0:
            i = order[0]
            keep.append(idx[i])
            xx1   = np.maximum(b[i, 0], b[order[1:], 0])
            yy1   = np.maximum(b[i, 1], b[order[1:], 1])
            xx2   = np.minimum(b[i, 2], b[order[1:], 2])
            yy2   = np.minimum(b[i, 3], b[order[1:], 3])
            w     = np.maximum(0, xx2 - xx1)
            h_    = np.maximum(0, yy2 - yy1)
            inter = w * h_
            area_i = (b[i, 2]-b[i, 0]) * (b[i, 3]-b[i, 1])
            area_o = (b[order[1:], 2]-b[order[1:], 0]) * (b[order[1:], 3]-b[order[1:], 1])
            iou    = inter / (area_i + area_o - inter + 1e-6)
            order  = order[np.where(iou <= iou_thresh)[0] + 1]

    return [detections[i] for i in sorted(keep)]


# ===== Drawing ==============================================================
def draw(frame, detections):
    """Draw bounding boxes and labels on frame."""
    for x1, y1, x2, y2, conf, cls in detections:
        name  = CLASS_NAMES[cls] if cls < len(CLASS_NAMES) else str(cls)
        color = COLORS.get(name, (255, 255, 255))
        label = f"{name} {conf:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, label, (x1, max(y1 - 8, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    return frame


# ===== Main =================================================================
def main(hef_path):
    try:
        from hailo_platform import (
            HEF, VDevice, HailoStreamInterface,
            InferVStreams, ConfigureParams, InputVStreamParams,
            OutputVStreamParams, FormatType
        )
    except ImportError:
        print("[ERROR] hailo_platform package not found.")
        print("  sudo apt install -y hailo-all")
        return

    try:
        from picamera2 import Picamera2
    except ImportError:
        print("[ERROR] picamera2 not found.")
        print("  sudo apt install -y python3-picamera2")
        return

    # Load HEF and configure device
    print(f"[INFO] Loading HEF: {hef_path}")
    hef    = HEF(hef_path)
    target = VDevice()
    network_groups = target.configure(hef, ConfigureParams.create_from_hef(
        hef, interface=HailoStreamInterface.PCIe))
    network_group        = network_groups[0]
    network_group_params = network_group.create_params()

    input_vstreams_params  = InputVStreamParams.make(
        network_group, format_type=FormatType.UINT8)
    output_vstreams_params = OutputVStreamParams.make(
        network_group, format_type=FormatType.FLOAT32)

    # Initialize camera
    cam = Picamera2()
    cfg = cam.create_video_configuration(
        main={"size": (CAMERA_W, CAMERA_H), "format": "RGB888"},
        controls={"FrameRate": 30},
    )                                            # 카메라 정방향 장착 → 회전(hflip/vflip) 없음
    cam.configure(cfg)
    cam.start()
    print("[INFO] Camera started. Press 'q' to quit.")

    # Inference loop
    with InferVStreams(network_group, input_vstreams_params,
                      output_vstreams_params) as infer_pipeline:
        with network_group.activate(network_group_params):
            while True:
                frame = cam.capture_array()  # RGB888

                input_data = {
                    hef.get_input_vstream_infos()[0].name: preprocess(frame)
                }
                output_data = infer_pipeline.infer(input_data)

                # 1) Model-level NMS
                raw_dets = postprocess(output_data, CAMERA_W, CAMERA_H)

                # 2) Color-based sign correction + cross-class NMS
                dets = correct_all_signs(frame, raw_dets)

                # 3) Draw
                frame_bgr = frame.copy()  # keep original for display
                frame_bgr = draw(frame_bgr, dets)

                cv2.putText(frame_bgr, f"Objects: {len(dets)}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (255, 255, 255), 2)

                cv2.imshow("Hailo Detection", frame_bgr)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    cam.stop()
    cam.close()
    cv2.destroyAllWindows()
    print("[INFO] Done.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--hef", default=HEF_PATH,
                        help="Path to HEF file (default: yolov11n.hef)")
    args = parser.parse_args()

    if not Path(args.hef).exists():
        print(f"[ERROR] HEF file not found: {args.hef}")
    else:
        main(args.hef)
