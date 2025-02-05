#!/usr/bin/env python3
"""
Module: feature_utils
Description: Contains the FeatureUtils class for keypoint detection,
             descriptor extraction, and matching using a FLANN-based matcher.
"""

import cv2
import numpy as np

class FeatureUtils:
    """
    Contains the feature detector and descriptor extractor and Feature Matching.
    """
    def __init__(self, ratio_test = 0.8):
        
        self.detector = cv2.xfeatures2d.SIFT_create()
        self.ratio_test = ratio_test
        self.index_params = dict(algorithm=1, trees=5)  # algorithm=1 corresponds to KDTree
        self.search_params = dict(checks=50)
        self.matcher = cv2.FlannBasedMatcher(self.index_params, self.search_params)
    

    def detect_and_compute(self, image):
        """
        Detect keypoints and compute descriptors for a given image.
        """
        keypoints, descriptors = self.detector.detectAndCompute(image, None)
        return keypoints, descriptors
    
    def match(self, des1, des2):
        """
        Matches descriptors between two images using k-nearest neighbors (k=2) and applies a ratio test.
        """
        raw_matches = self.matcher.knnMatch(des1, des2, k=2)
        good_matches = []
        for match_pair in raw_matches:
            if len(match_pair) != 2:
                continue
            m, n = match_pair
            if m.distance < self.ratio_test * n.distance:
                good_matches.append(m)
        return good_matches
