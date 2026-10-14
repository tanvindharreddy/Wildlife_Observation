import cv2
import json
import time
import os
from ultralytics import YOLO

# ==================================================
# 1. PROJECT PATHS
# ==================================================

PYTHON_FOLDER = os.path.dirname(os.path.abspath(__file__))
PROJECT_FOLDER = os.path.dirname(PYTHON_FOLDER)

DATA_FOLDER = os.path.join(PROJECT_FOLDER, "data")
JSON_FILE = os.path.join(DATA_FOLDER, "vision_data.json")

GENERAL_MODEL_FILE = os.path.join(PYTHON_FOLDER, "yolo11n.pt")

WILDLIFE_MODEL_FILE = os.path.join(
    PROJECT_FOLDER,
    "wildlife_training",
    "wildlife_yolo11n",
    "weights",
    "best.pt",
)

os.makedirs(DATA_FOLDER, exist_ok=True)

# Your phone camera URL
PHONE_URL = "http://10.21.195.252:8080/video"

# ==================================================
# 2. SETTINGS
# ==================================================

LOST_TIMEOUT = 3.0

OBSTACLE_CLASSES = {
    "chair", "couch", "bed", "dining table",
    "potted plant", "bench", "suitcase",
    "backpack", "bicycle", "motorcycle",
    "car", "bus", "truck", "tv",
    "refrigerator", "toilet"
}

# Wildlife classes:
# buffalo, elephant, rhino, zebra

# ==================================================
# 3. LOAD BOTH MODELS
# ==================================================

print("Loading general YOLO11n model...")
general_model = YOLO(GENERAL_MODEL_FILE)

print("Loading wildlife model...")
wildlife_model = YOLO(WILDLIFE_MODEL_FILE)

print("Both models loaded!")
print("Wildlife classes:", wildlife_model.names)

# ==================================================
# 4. CONNECT TO PHONE CAMERA
# ==================================================

cap = cv2.VideoCapture(PHONE_URL)

if not cap.isOpened():
    print("ERROR: Cannot connect to phone camera.")
    print("Check phone IP, Wi-Fi and IP Webcam.")
    raise SystemExit

print("Phone camera connected!")
print("Click RESTART DETECTION or press R to reset.")
print("Press Q to quit.")

# ==================================================
# 5. TARGET TRACKING VARIABLES
# ==================================================

target_id = None
target_class = None
last_target_seen = None

# State for the clickable restart button
ui_state = {
    "restart": False,
    "button": (0, 0, 0, 0)
}


def handle_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        x1, y1, x2, y2 = ui_state["button"]

        if x1 <= x <= x2 and y1 <= y <= y2:
            ui_state["restart"] = True


WINDOW_NAME = "Wildlife + Obstacle Detection"

cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
cv2.setMouseCallback(WINDOW_NAME, handle_mouse)

# ==================================================
# 6. MAIN LOOP
# ==================================================

while True:

    ret, frame = cap.read()

    if not ret:
        print("Cannot read camera frame.")
        time.sleep(0.1)
        continue

    frame_height, frame_width = frame.shape[:2]

    # ----------------------------------------------
    # A. TRACK WILDLIFE
    # ----------------------------------------------

    wildlife_results = wildlife_model.track(
        frame,
        persist=True,
        tracker="bytetrack.yaml",
        conf=0.25,
        verbose=False
    )

    wildlife_result = wildlife_results[0]

    # ----------------------------------------------
    # B. DETECT GENERAL OBJECTS
    # ----------------------------------------------

    general_results = general_model.predict(
        frame,
        imgsz=640,
        conf=0.35,
        verbose=False
    )

    general_result = general_results[0]

    animals = []
    obstacles = []

    # ----------------------------------------------
    # C. COLLECT WILDLIFE DETECTIONS
    # ----------------------------------------------

    boxes = wildlife_result.boxes

    if boxes is not None and len(boxes) > 0:

        for i in range(len(boxes)):

            class_id = int(boxes.cls[i].item())
            confidence = float(boxes.conf[i].item())

            class_name = wildlife_model.names[class_id]

            x1, y1, x2, y2 = boxes.xyxy[i].cpu().tolist()

            x1, y1, x2, y2 = map(
                int, (x1, y1, x2, y2)
            )

            box_width = max(1, x2 - x1)
            box_height = max(1, y2 - y1)

            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2

            track_id = None

            if boxes.id is not None:
                track_id = int(boxes.id[i].item())

            animals.append({
                "track_id": track_id,
                "class": class_name,
                "class_id": class_id,
                "confidence": confidence,
                "x1": x1,
                "y1": y1,
                "x2": x2,
                "y2": y2,
                "width": box_width,
                "height": box_height,
                "pixel_x": center_x,
                "pixel_y": center_y,
                "area": box_width * box_height
            })

    # ----------------------------------------------
    # D. COLLECT POTENTIAL OBSTACLES
    # ----------------------------------------------

    general_boxes = general_result.boxes

    if general_boxes is not None and len(general_boxes) > 0:

        for i in range(len(general_boxes)):

            class_id = int(general_boxes.cls[i].item())
            class_name = general_model.names[class_id]

            if class_name not in OBSTACLE_CLASSES:
                continue

            confidence = float(general_boxes.conf[i].item())

            x1, y1, x2, y2 = (
                general_boxes.xyxy[i].cpu().tolist()
            )

            x1, y1, x2, y2 = map(
                int, (x1, y1, x2, y2)
            )

            box_width = max(1, x2 - x1)
            box_height = max(1, y2 - y1)

            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2

            # Rough image indicators, NOT actual metres
            normalized_x = (
                (center_x - frame_width / 2)
                / (frame_width / 2)
            )

            estimated_x = normalized_x * 2.0

            estimated_y = (
                1.0 - box_height / frame_height
            ) * 4.0

            obstacles.append({
                "class": class_name,
                "confidence": confidence,
                "x": float(estimated_x),
                "y": float(estimated_y),
                "pixel_x": center_x,
                "pixel_y": center_y,
                "width": box_width,
                "height": box_height
            })

            cv2.rectangle(
                frame,
                (x1, y1),
                (x2, y2),
                (0, 165, 255),
                2
            )

            cv2.putText(
                frame,
                f"Obstacle: {class_name}",
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 165, 255),
                2
            )

    # ----------------------------------------------
    # E. SELECT INITIAL WILDLIFE TARGET
    # ----------------------------------------------

    tracked_animals = [
        animal for animal in animals
        if animal["track_id"] is not None
    ]

    if target_id is None and tracked_animals:

        # Select the largest visible tracked animal
        selected = max(
            tracked_animals,
            key=lambda animal: animal["area"]
        )

        target_id = selected["track_id"]
        target_class = selected["class"]
        last_target_seen = time.time()

        print(
            f"ANIMAL TARGET LOCKED: "
            f"{target_class}, ID {target_id}"
        )

    # ----------------------------------------------
    # F. FOLLOW THE LOCKED ANIMAL
    # ----------------------------------------------

    selected_target = None

    if target_id is not None:

        for animal in animals:

            if (
                animal["track_id"] == target_id
                and animal["class"] == target_class
            ):
                selected_target = animal
                break

    # ----------------------------------------------
    # G. AUTOMATIC SEARCH / REACQUISITION
    # ----------------------------------------------

    if (
        selected_target is None
        and target_id is not None
        and last_target_seen is not None
        and time.time() - last_target_seen > LOST_TIMEOUT
    ):

        print(
            f"TARGET LOST: {target_class}, ID {target_id}. "
            "Returning to search."
        )

        target_id = None
        target_class = None
        last_target_seen = None

        if tracked_animals:

            selected_target = max(
                tracked_animals,
                key=lambda animal: animal["area"]
            )

            target_id = selected_target["track_id"]
            target_class = selected_target["class"]
            last_target_seen = time.time()

            print(
                f"AUTO-LOCKED NEW TARGET: "
                f"{target_class}, ID {target_id}"
            )

    # ----------------------------------------------
    # H. CREATE TARGET DATA
    # ----------------------------------------------

    target = {
        "found": False,
        "class": target_class,
        "x": None,
        "y": None,
        "pixel_x": None,
        "pixel_y": None,
        "width": None,
        "height": None,
        "target_id": target_id,
        "target_lost": target_id is not None
    }

    if selected_target is not None:

        animal = selected_target

        last_target_seen = time.time()

        normalized_x = (
            (animal["pixel_x"] - frame_width / 2)
            / (frame_width / 2)
        )

        estimated_x = normalized_x * 2.0

        estimated_y = (
            1.0 - animal["height"] / frame_height
        ) * 4.0

        target = {
            "found": True,
            "class": animal["class"],
            "x": float(estimated_x),
            "y": float(estimated_y),
            "pixel_x": animal["pixel_x"],
            "pixel_y": animal["pixel_y"],
            "width": animal["width"],
            "height": animal["height"],
            "target_id": target_id,
            "target_lost": False
        }

        x1 = animal["x1"]
        y1 = animal["y1"]
        x2 = animal["x2"]
        y2 = animal["y2"]

        cv2.rectangle(
            frame,
            (x1, y1),
            (x2, y2),
            (0, 255, 0),
            3
        )

        cv2.putText(
            frame,
            f"LOCKED ANIMAL: {target_class} ID {target_id}",
            (x1, max(25, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2
        )

    else:

        if target_id is not None:
            status = "SEARCHING FOR LOCKED ANIMAL"
        else:
            status = "SEARCHING FOR WILDLIFE - AUTO-LOCK ENABLED"

        cv2.putText(
            frame,
            status,
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2
        )

    # ----------------------------------------------
    # I. WRITE JSON FOR MATLAB
    # ----------------------------------------------

    # Keep "human" as a compatibility alias for old MATLAB code.

    data = {
        "target": target,
        "human": target,

        "animals": [
            {
                "track_id": animal["track_id"],
                "class": animal["class"],
                "confidence": animal["confidence"],
                "pixel_x": animal["pixel_x"],
                "pixel_y": animal["pixel_y"],
                "width": animal["width"],
                "height": animal["height"]
            }
            for animal in animals
        ],

        "obstacles": obstacles,

        "frame": {
            "width": frame_width,
            "height": frame_height
        },

        "timestamp": time.time()
    }

    temp_file = JSON_FILE + ".tmp"

    try:

        with open(temp_file, "w", encoding="utf-8") as f:
            json.dump(data, f)

        os.replace(temp_file, JSON_FILE)

    except OSError as error:
        print("JSON writing error:", error)

    # ----------------------------------------------
    # J. DISPLAY STATUS
    # ----------------------------------------------

    cv2.putText(
        frame,
        f"Wildlife tracks: {len(animals)} | "
        f"Obstacles: {len(obstacles)}",
        (20, frame_height - 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2
    )

    cv2.putText(
        frame,
        "Auto search enabled | R: reset | Q: quit",
        (20, frame_height - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2
    )

    # ----------------------------------------------
    # K. CLICKABLE RESTART BUTTON
    # ----------------------------------------------

    button_x1 = max(10, frame_width - 235)
    button_y1 = 10
    button_x2 = max(button_x1 + 150, frame_width - 10)
    button_y2 = 48

    ui_state["button"] = (
        button_x1,
        button_y1,
        button_x2,
        button_y2
    )

    cv2.rectangle(
        frame,
        (button_x1, button_y1),
        (button_x2, button_y2),
        (40, 40, 220),
        -1
    )

    cv2.rectangle(
        frame,
        (button_x1, button_y1),
        (button_x2, button_y2),
        (255, 255, 255),
        2
    )

    cv2.putText(
        frame,
        "RESTART DETECTION",
        (button_x1 + 8, button_y1 + 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (255, 255, 255),
        2
    )

    cv2.imshow(WINDOW_NAME, frame)

    # ----------------------------------------------
    # L. KEYBOARD / BUTTON CONTROLS
    # ----------------------------------------------

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break

    elif key == ord("r") or ui_state["restart"]:

        target_id = None
        target_class = None
        last_target_seen = None

        ui_state["restart"] = False

        print(
            "Manual restart: target cleared. "
            "Automatic detection will select a target again."
        )

# ==================================================
# 7. CLEANUP
# ==================================================

cap.release()
cv2.destroyAllWindows()

print("Wildlife monitoring system stopped.")