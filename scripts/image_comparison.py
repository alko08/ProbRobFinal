# Import necessary libraries
import cv2
import numpy as np
from skimage.metrics import structural_similarity as ssim
from tensorflow.keras.applications.vgg16 import VGG16, preprocess_input
from tensorflow.keras.preprocessing import image
from tensorflow.keras.models import Model
from scipy.spatial.distance import cosine
import sys
import os

# Function to calculate SSIM
def calculate_ssim(imageA, imageB, window_name_A, window_name_B):
    # Resize the smaller image to match the larger image's dimensions
    if imageA.shape[:2] != imageB.shape[:2]:
        heightA, widthA = imageA.shape[:2]
        heightB, widthB = imageB.shape[:2]
        
        if heightA * widthA < heightB * widthB:
            imageA = cv2.resize(imageA, (widthB, heightB))
        else:
            imageB = cv2.resize(imageB, (widthA, heightA))

    # Convert images to grayscale
    grayA = cv2.cvtColor(imageA, cv2.COLOR_BGR2GRAY)
    grayB = cv2.cvtColor(imageB, cv2.COLOR_BGR2GRAY)

    cv2.imshow(window_name_A, grayA)
    cv2.waitKey()
    cv2.imshow(window_name_B, grayB)
    cv2.waitKey()
    
    # Compute SSIM between the two images
    # score, _ = ssim(grayA, grayB, full=True)
    score, full_sim_image = ssim(grayA, grayB, full=True)
    return score, full_sim_image

# Function to extract features using VGG16 and calculate cosine similarity
def calculate_feature_similarity(image_path1, image_path2):
    # Load pre-trained VGG16 model
    base_model = VGG16(weights='imagenet', include_top=False)
    model = Model(inputs=base_model.input, outputs=base_model.get_layer('block5_pool').output)
    
    # Function to preprocess image for VGG16
    def preprocess_image(image_path):
        img = image.load_img(image_path, target_size=(224, 224))
        img_data = image.img_to_array(img)
        img_data = np.expand_dims(img_data, axis=0)
        img_data = preprocess_input(img_data)
        return img_data
    
    # Preprocess the two images
    img1 = preprocess_image(image_path1)
    img2 = preprocess_image(image_path2)
    
    # Extract features
    features_img1 = model.predict(img1)
    features_img2 = model.predict(img2)
    
    # Flatten features and calculate cosine similarity
    features_img1 = features_img1.flatten()
    features_img2 = features_img2.flatten()
    similarity = 1 - cosine(features_img1, features_img2)
    
    return similarity

imagePathA = sys.argv[1]
imagePathB = sys.argv[2]
imageA = cv2.imread(imagePathA)
imageB = cv2.imread(imagePathB)
window_name_A = os.path.splitext(os.path.basename(imagePathA))[0]
window_name_B = os.path.splitext(os.path.basename(imagePathB))[0]

# Calculate SSIM
ssim_score, full_sim_image = calculate_ssim(imageA, imageB, window_name_A, window_name_B)
print(f"SSIM score: {ssim_score}")
cv2.imshow('Learned Similarity Map', full_sim_image)

# Calculate feature-based similarity using VGG16
feature_similarity = calculate_feature_similarity(imagePathA, imagePathB)
print(f"Feature-based similarity (Cosine similarity): {feature_similarity}")
cv2.waitKey()