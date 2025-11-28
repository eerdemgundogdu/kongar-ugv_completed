# YOLOv8 Training for Teknofest Parkour

## Quick Start

### 1. Install Dependencies
```bash
pip install ultralytics roboflow
```

### 2. Prepare Dataset

#### Option A: Use Roboflow (Recommended)
1. Go to [roboflow.com](https://roboflow.com)
2. Create a project
3. Upload your images
4. Label obstacles, ramps, traffic signs, etc.
5. Export in **YOLOv8 format**
6. Download and extract to `datasets/teknofest_parkour/`

#### Option B: Manual Labeling
1. Install LabelImg: `pip install labelImg`
2. Run: `labelImg`
3. Label images and save in YOLO format
4. Organize:
   ```
   datasets/teknofest_parkour/
   ├── images/
   │   ├── train/
   │   └── val/
   └── labels/
       ├── train/
       └── val/
   ```

### 3. Train
```bash
python train_yolo.py
```

### 4. Test
```bash
python -c "from ultralytics import YOLO; model = YOLO('runs/detect/teknofest_model/weights/best.pt'); model.predict('test_image.jpg', save=True)"
```

## Dataset Classes

The model is configured to detect:
- **obstacle**: General obstacles
- **ramp**: Ramps for climbing
- **traffic_sign**: Traffic signs
- **parking_spot**: Parking areas
- **pedestrian**: People

Edit `teknofest_parkour.yaml` to add/remove classes.

## Training Tips

1. **Collect diverse data**: Different lighting, angles, distances
2. **Augmentation**: Roboflow automatically adds augmentations
3. **Balance classes**: Try to have similar number of examples per class
4. **Start small**: Train on 50-100 images first to test
5. **Monitor**: Check `runs/detect/teknofest_model/` for training plots

## Deployment

After training, copy the model:
```bash
cp runs/detect/teknofest_model/weights/best.pt ../ros2_ws/src/ugv_vision/models/
```

Then update `object_detector.py`:
```python
self.model = YOLO('models/best.pt')  # Use your trained model
```

## GPU Training (Recommended)

If you don't have a GPU, use Google Colab:
1. Upload this folder to Google Drive
2. Open Colab notebook
3. Run:
   ```python
   !pip install ultralytics
   from google.colab import drive
   drive.mount('/content/drive')
   %cd /content/drive/MyDrive/training
   !python train_yolo.py
   ```
