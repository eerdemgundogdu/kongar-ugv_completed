from ultralytics import YOLO

# Initialize YOLOv8 model
model = YOLO("yolov8n.pt")  # Load a pretrained model (recommended for training)

# Train the model
# data: Path to data.yaml file describing the dataset
# epochs: Number of training epochs
# imgsz: Image size
# batch: Batch size
# device: '0' for GPU, 'cpu' for CPU
results = model.train(
    data="teknofest_parkour.yaml", 
    epochs=100, 
    imgsz=640, 
    batch=16, 
    device=0, # Set to 'cpu' if no GPU
    name="teknofest_model"
)

# Validate the model
metrics = model.val()

# Export the model
success = model.export(format="onnx")
