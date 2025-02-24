import cv2
import torch
import numpy as np
from transformers import AutoImageProcessor, AutoModelForDepthEstimation

# Initialize the Depth Anything model
model_name = "Intel/dpt-hybrid-midas"
processor = AutoImageProcessor.from_pretrained(model_name)
model = AutoModelForDepthEstimation.from_pretrained(model_name)

# Move model to GPU if available
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
# device = torch.device("cpu")
model.to(device)

# Initialize video capture
cap = cv2.VideoCapture(0)

# Get video properties
fps = int(cap.get(cv2.CAP_PROP_FPS))
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

def print_center_depth(depth_map):
    center_x, center_y = depth_map.shape[1] // 2, depth_map.shape[0] // 2
    center_depth = depth_map[center_y, center_x]
    print(f"Center depth: {center_depth}")
# Initialize video writer
# fourcc = cv2.VideoWriter_fourcc(*'mp4v')
# out = cv2.VideoWriter('output.mp4', fourcc, fps, (width * 2, height))

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Preprocess the frame
    inputs = processor(images=frame, return_tensors="pt").to(device)

    # Perform inference
    with torch.no_grad():
        outputs = model(**inputs)
        predicted_depth = outputs.predicted_depth

    # Normalize depth map for visualization
    depth_map = torch.nn.functional.interpolate(
        predicted_depth.unsqueeze(1),
        size=frame.shape[:2],
        mode="bicubic",
        align_corners=False,
    ).squeeze().cpu().numpy()
    
    depth_map = (depth_map - depth_map.min()) / (depth_map.max() - depth_map.min())
    depth_map = (depth_map * 255).astype(np.uint8)
    depth_map_color = cv2.applyColorMap(depth_map, cv2.COLORMAP_PLASMA)

    print_center_depth(depth_map)

    # Combine frame and depth map side by side
    combined_image = np.hstack((frame, depth_map_color))

    # Write the frame to the output video
    # out.write(combined_image)

    # Display the results 
    cv2.namedWindow('RGB and Depth', cv2.WINDOW_NORMAL)
    cv2.imshow('RGB and Depth', combined_image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
