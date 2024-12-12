import cv2
import numpy as np
from skimage.metrics import structural_similarity as ssim
from PIL import Image

def load_map(file_path):
    """
    Load a map image from the given file path.
    If the file is a PGM, it will be converted to PNG using Pillow.
    
    Args:
        file_path (str): Path to the map image.
    
    Returns:
        numpy.ndarray: Grayscale image array of the map.
    """
    # Check if the file is a PGM file and convert
    if file_path.lower().endswith('.pgm'):
        with Image.open(file_path) as img:
            converted_path = file_path.replace('.pgm', '.png')
            img.save(converted_path)
            file_path = converted_path
    
    # Load the image using OpenCV
    map_image = cv2.imread(file_path, cv2.IMREAD_GRAYSCALE)
    if map_image is None:
        raise ValueError(f"Failed to load image at {file_path}")
    return map_image

def resize_maps(map1, map2):
    """
    Resize the maps to ensure they have the same dimensions.
    Args:
        map1 (numpy.ndarray): First map image.
        map2 (numpy.ndarray): Second map image.
    Returns:
        tuple: Resized map1 and map2.
    """
    height, width = map1.shape
    resized_map2 = cv2.resize(map2, (width, height), interpolation=cv2.INTER_AREA)
    return map1, resized_map2

def compute_error(map1, map2):
    """
    Compute the error score between two maps.
    Args:
        map1 (numpy.ndarray): First map image.
        map2 (numpy.ndarray): Second map image.
    Returns:
        float: Error score based on Mean Squared Error (MSE).
    """
    mse = np.mean((map1.astype("float") - map2.astype("float")) ** 2)
    return mse

def compute_accuracy(map, ground_truth):
    """
    Compute the accuracy score of a map compared to a ground truth map.
    Args:
        map (numpy.ndarray): Map image.
        ground_truth (numpy.ndarray): Ground truth map image.
    Returns:
        float: Accuracy score based on Structural Similarity Index (SSIM).
    """
    score, _ = ssim(map, ground_truth, full=True)
    return score

if __name__ == "__main__":
    # File paths for the maps and ground truth
    map1_path = "path_to_map1.png"
    map2_path = "path_to_map2.png"
    ground_truth_path = "path_to_ground_truth.png"
    
    # Load images
    map1 = load_map(map1_path)
    map2 = load_map(map2_path)
    ground_truth = load_map(ground_truth_path)
    
    # Resize maps to match dimensions
    map1, map2 = resize_maps(map1, map2)
    map1, ground_truth = resize_maps(map1, ground_truth)
    
    # Compute metrics
    error_score = compute_error(map1, map2)
    accuracy_score = compute_accuracy(map1, ground_truth)
    
    # Print results
    print(f"Error Score (MSE) between map1 and map2: {error_score:.4f}")
    print(f"Accuracy Score (SSIM) of map1 to ground truth: {accuracy_score:.4f}")
