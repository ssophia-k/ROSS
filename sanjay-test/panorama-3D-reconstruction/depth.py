"""
Step 2 — Generate depth maps using MiDaS DPT_Hybrid.
Runs on CPU (slow but works) or CUDA GPU if available.
"""

from transformers import DPTForDepthEstimation, DPTImageProcessor
import torch
import cv2
import numpy as np
import glob
import os

os.makedirs("data/depth", exist_ok=True)

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Using device: {device}")

print("Loading DPT via transformers...")
# takes a while ~10 minutes to process 150 frames
processor = DPTImageProcessor.from_pretrained("Intel/dpt-hybrid-midas")
model = DPTForDepthEstimation.from_pretrained("Intel/dpt-hybrid-midas")
model.to(device)
model.eval()

color_files = sorted(glob.glob("data/color/*.jpg"))
print(f"Processing {len(color_files)} frames...\n")

for i, fpath in enumerate(color_files):
    fname = os.path.basename(fpath).replace(".jpg", ".png")
    out_path = f"data/depth/{fname}"

    if os.path.exists(out_path):
        print(f"  [{i+1}/{len(color_files)}] Skipping {fname}")
        continue

    img = cv2.imread(fpath)
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # Prepare input
    inputs = processor(images=img_rgb, return_tensors="pt").to(device)

    with torch.no_grad():
        outputs = model(**inputs)
        prediction = outputs.predicted_depth  # shape: (1, H, W)

    # Resize back to original frame size
    prediction = torch.nn.functional.interpolate(
        prediction.unsqueeze(1),
        size=img_rgb.shape[:2],
        mode="bicubic",
        align_corners=False
    ).squeeze().cpu().numpy()

    depth_min = prediction.min()
    depth_max = prediction.max()
    depth_norm = ((prediction - depth_min) / (depth_max - depth_min) * 65535).astype(np.uint16)

    cv2.imwrite(out_path, depth_norm)
    print(f"  [{i+1}/{len(color_files)}] {fname}  "
          f"range [{prediction.min():.2f}, {prediction.max():.2f}]")

print(f"\nDone. Depth maps saved to data/depth/")