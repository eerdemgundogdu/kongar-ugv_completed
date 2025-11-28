import os
import requests

def download_yolo_model():
    url = "https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt"
    target_path = os.path.join(os.path.dirname(__file__), "yolov8n.pt")
    
    if os.path.exists(target_path):
        print(f"Model already exists at {target_path}")
        return

    print(f"Downloading YOLOv8n model to {target_path}...")
    try:
        response = requests.get(url, stream=True)
        response.raise_for_status()
        
        with open(target_path, 'wb') as f:
            for chunk in response.iter_content(chunk_size=8192):
                f.write(chunk)
        print("Download complete!")
    except Exception as e:
        print(f"Failed to download model: {e}")
        print("Please manually download 'yolov8n.pt' and place it in this directory.")

if __name__ == "__main__":
    download_yolo_model()
