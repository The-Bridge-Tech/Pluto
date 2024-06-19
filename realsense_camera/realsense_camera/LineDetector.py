# Matthew Lauriault
# Detect line attributes from an image
# 6/19/24


# IMPORTS
import cv2
import numpy as np


class Line:
        """Struct to hold line attributes."""
        def __init__(self, x1: int, y1: int, x2: int, y2: int, theta: float):
                self.x1 = x1,
                self.y1 = y1
                self.x2 = x2
                self.y2 = y2
                self.theta = theta
        def getDegrees(self):
                return np.degrees(self.theta)
        def getCoord1(self):
                return (self.x1, self.y1)
        def getCoord2(self):
                return (self.x2, self.y2)
        def divideImage(self, image: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
                """Divide image into regions based on line. Return the divided image regions."""
                # Create left and right masks to divide the image
                height, width = image.shape[:2]
                left_mask = np.zeros((height, width), dtype=np.uint8)
                right_mask = np.zeros((height, width), dtype=np.uint8)
                # Fill left and right regions based on the line
                cv2.fillPoly(left_mask, [np.array([
                    (0, 0),
                    self.getCoord1(),
                    self.getCoord2(),
                    (0, height)
                ])], 255)
                cv2.fillPoly(right_mask, [np.array([
                    (width, 0),
                    self.getCoord1(),
                    self.getCoord2(),
                    (width, height)
                ])], 255)
                # Get the left and right regions using the respective masks on the image
                left_region = cv2.bitwise_and(image, image, mask=left_mask)
                right_region = cv2.bitwise_and(image, image, mask=right_mask)
                return left_region, right_region
        def __str__(self) -> str:
                return f"Line: {self.getCoord1()}, {self.getCoord2()}, {self.getDegrees()}°"
        def __repr__(self) -> str:
                return self.__str__()

        
def detectLine(image: np.ndarray) -> Line:
        """Detect line from numpy image array. Returns None on failure."""
        # Convert image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Get the edges from the image
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)
        # Get the lines from the edges
        lines = cv2.HoughLines(edges, 1, np.pi/180, 200)
        if lines is not None:
                for rho, theta in lines[0]:
                        a = np.cos(theta)
                        b = np.sin(theta)
                        x0 = a * rho
                        y0 = b * rho
                        x1 = int(x0 + 1000 * (-b))
                        y1 = int(y0 + 1000 * a)
                        x2 = int(x0 - 1000 * (-b))
                        y2 = int(y0 - 1000 * a)
                        return Line(x1, y1, x2, y2, theta)
        return None


# TESTS

def testDetectLine():
        