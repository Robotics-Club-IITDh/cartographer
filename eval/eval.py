import yaml
import cv2
import numpy as np
import os
import sys

def load_ros_map(yaml_path):
    """Loads a ROS map using its YAML and PGM image."""
    with open(yaml_path, 'r') as f:
        meta = yaml.safe_load(f)
    
    # Get image path relative to the yaml file
    img_name = meta['image']
    img_path = os.path.join(os.path.dirname(yaml_path), img_name)
    
    # Read image in grayscale
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Could not load image: {img_path}")
        
    return img, meta

def convert_to_occupancy(img, meta):
    """Converts a standard grayscale PGM image to standard Nav2 grid values:
       0 = Free, 100 = Occupied, -1 = Unknown
    """
    occ = np.zeros_like(img, dtype=np.int8)
    
    # Extract thresholds from yaml
    free_thresh = meta.get('free_thresh', 0.196) * 255
    occupied_thresh = meta.get('occupied_thresh', 0.65) * 255
    negate = meta.get('negate', 0)
    
    if negate:
        # Inverted logic
        occ[img >= occupied_thresh] = 0     # Free
        occ[img <= free_thresh] = 100       # Occupied
        occ[(img > free_thresh) & (img < occupied_thresh)] = -1 # Unknown
    else:
        # Standard ROS logic
        occ[img >= (255 - free_thresh)] = 0       # Free
        occ[img <= (255 - occupied_thresh)] = 100   # Occupied
        occ[(img > (255 - occupied_thresh)) & (img < (255 - free_thresh))] = -1
        
    return occ

def align_maps(gt_occ, gt_meta, test_occ, test_meta):
    """Aligns the test map frame onto the ground truth map frame using world coordinates."""
    gt_res = gt_meta['resolution']
    test_res = test_meta['resolution']
    
    # Confirm resolutions match for simple array math
    if not np.isclose(gt_res, test_res, atol=1e-4):
        print(f"[Warning] Resolution mismatch! Rescaling test map from {test_res} to {gt_res}")
        scale = test_res / gt_res
        test_occ = cv2.resize(test_occ.astype(np.float32), (0,0), fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST).astype(np.int8)

    # Real world coordinates of the bottom-left corner [X, Y, Yaw]
    gt_origin = gt_meta['origin']
    test_origin = test_meta['origin']
    
    # Calculate offset in meters, convert to pixel offsets on the GT canvas
    dx_meters = test_origin[0] - gt_origin[0]
    dy_meters = test_origin[1] - gt_origin[1]
    
    pixel_offset_x = int(round(dx_meters / gt_res))
    pixel_offset_y = int(round(dy_meters / gt_res)) # Image coordinates grow down, world grows up

    # Create an empty template mirroring the shape of our ground truth canvas
    aligned_test = np.full_like(gt_occ, -1, dtype=np.int8) # Default everything to Unknown (-1)

    # Calculate overlapping regions
    gt_h, gt_w = gt_occ.shape
    test_h, test_w = test_occ.shape

    # Determine boundaries inside Ground Truth space
    gt_x_start = max(0, pixel_offset_x)
    gt_y_start = max(0, pixel_offset_y)
    gt_x_end = min(gt_w, pixel_offset_x + test_w)
    gt_y_end = min(gt_h, pixel_offset_y + test_h)

    # Corresponding boundaries inside Test Space
    test_x_start = max(0, -pixel_offset_x)
    test_y_start = max(0, -pixel_offset_y)
    test_x_end = test_x_start + (gt_x_end - gt_x_start)
    test_y_end = test_y_start + (gt_y_end - gt_y_start)

    # Slice and copy the data over
    if (gt_x_end > gt_x_start) and (gt_y_end > gt_y_start):
        aligned_test[gt_y_start:gt_y_end, gt_x_start:gt_x_end] = test_occ[test_y_start:test_y_end, test_x_start:test_x_end]

    return aligned_test

def evaluate_metrics(gt, test):
    """Computes mathematical error profiles between the maps."""
    # Mask out areas where both maps are marked "unknown (-1)" so it doesn't inflate accuracy scores
    valid_mask = (gt != -1) & (test != -1)
    
    if np.sum(valid_mask) == 0:
        return {"Error": "No overlapping mapped areas discovered!"}

    gt_valid = gt[valid_mask]
    test_valid = test[valid_mask]

    # 1. Mean Squared Error (MSE)
    mse = np.mean((gt_valid - test_valid) ** 2)

    # 2. Obstacle Accuracy Metrics (Intersection over Union)
    gt_obstacles = (gt == 100)
    test_obstacles = (test == 100)
    
    intersection = np.sum(gt_obstacles & test_obstacles)
    union = np.sum(gt_obstacles | test_obstacles)
    iou = (intersection / union * 100) if union > 0 else 0.0

    # 3. Structural Classification Match Percentage
    exact_matches = np.sum(gt_valid == test_valid)
    accuracy = (exact_matches / len(gt_valid)) * 100

    return {
        "Total Overlapping Evaluated Cells": len(gt_valid),
        "Map Agreement Accuracy": f"{accuracy:.2f}%",
        "Obstacle Intersection-over-Union (IoU)": f"{iou:.2f}%",
        "Mean Squared Error (Lower is better)": f"{mse:.4f}"
    }

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 eval.py <ground_truth.yaml> <student_map.yaml>")
        sys.exit(1)
        
    gt_yaml = sys.argv[1]
    test_yaml = sys.argv[2]
    
    print("[1/3] Loading map textures and YAML configurations...")
    gt_img, gt_meta = load_ros_map(gt_yaml)
    test_img, test_meta = load_ros_map(test_yaml)
    
    print("[2/3] Normalizing occupancy grid layouts...")
    gt_occ = convert_to_occupancy(gt_img, gt_meta)
    test_occ = convert_to_occupancy(test_img, test_meta)
    
    # Flip arrays vertically because ROS maps assume origin is bottom-left
    # but OpenCV images treat top-left as [0,0]
    gt_occ = np.flipud(gt_occ)
    test_occ = np.flipud(test_occ)
    
    print("[3/3] Aligning coordinate origins and computing quality metrics...")
    aligned_test = align_maps(gt_occ, gt_meta, test_occ, test_meta)
    
    results = evaluate_metrics(gt_occ, aligned_test)
    
    print("\n--- MAP EVALUATION SUMMARY ---")
    for metric, value in results.items():
        print(f"{metric:<40}: {value}")