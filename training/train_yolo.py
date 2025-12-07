from ultralytics import YOLO
import os

# Create dataset directories if missing
dirs = [
    "datasets/teknofest_parkour/images/train",
    "datasets/teknofest_parkour/images/val", 
    "datasets/teknofest_parkour/labels/train",
    "datasets/teknofest_parkour/labels/val"
]
for d in dirs:
    os.makedirs(d, exist_ok=True)

model = YOLO("yolov8n.pt")

results = model.train(
    data="teknofest_parkour.yaml", 
    epochs=100, 
    imgsz=640, 
    batch=16, 
    device=0,
    name="ugv_detector"
)

metrics = model.val()
success = model.export(format="onnx")
