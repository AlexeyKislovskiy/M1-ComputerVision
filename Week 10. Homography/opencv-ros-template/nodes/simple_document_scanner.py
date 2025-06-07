#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

class DocumentScanner:
    def __init__(self):
        self.bridge = CvBridge()
        self.current_frame = None
        self.captured_image = None
        self.scanning = False
        
        rospy.init_node('simple_document_scanner', anonymous=True)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        
    def image_callback(self, msg):
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Ошибка преобразования изображения: {e}")

    def capture_frame(self):
        if self.current_frame is not None:
            self.captured_image = self.current_frame.copy()
            return True
        return False

    def run(self):
        print("Инструкция:")
        print("1. Нажмите 's' для захвата кадра с камеры")
        print("2. Нажмите 'q' для выхода")
        
        while not rospy.is_shutdown():
            if self.current_frame is not None:
                cv2.imshow("Camera", self.current_frame)
                
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('s'):
                if self.capture_frame():
                    self.process_document()
            
            elif key == ord('q'):
                break
       
        cv2.destroyAllWindows()

    def process_document(self):
        if self.captured_image is None:
            return
            
        selector = CornerSelector(self.captured_image, "Select Document Corners")
        selected_corners = selector.select_corners()
        
        if selected_corners is not None:
            aligned_img = align_document(self.captured_image, selected_corners)
            show_and_save_results(self.captured_image, aligned_img)

class CornerSelector:
    def __init__(self, image, window_name="Select 4 corners"):
        self.image = image.copy()
        self.display_image = image.copy()
        self.window_name = window_name
        self.corners = []
        self.is_completed = False

    def select_corners(self):
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.imshow(self.window_name, self.display_image)
        cv2.waitKey(1)

        cv2.setMouseCallback(self.window_name, self._mouse_callback)

        print("Инструкция:")
        print("1. Кликните ЛКМ по 4 углам документа в порядке: верхний левый, верхний правый, нижний правый, нижний левый")
        print("2. Для выхода нажмите ESC")

        while True:
            cv2.imshow(self.window_name, self.display_image)
            key = cv2.waitKey(20) & 0xFF

            if key == 27 or self.is_completed:
                break

        cv2.destroyAllWindows()
        return np.array(self.corners, dtype=np.float32) if self.is_completed else None

    def _mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(self.corners) < 4:
            self.corners.append((x, y))
            print(f"Точка {len(self.corners)}: ({x}, {y})")

            self.display_image = self.image.copy()
            for i, (cx, cy) in enumerate(self.corners, 1):
                cv2.circle(self.display_image, (cx, cy), 10, (0, 0, 255), -1)
                cv2.putText(self.display_image, str(i), (cx + 15, cy - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            if len(self.corners) == 4:
                self.is_completed = True


def align_document(image, src_points, width=420, height=594):
    dst_points = np.array([
        [0, 0],
        [width - 1, 0],
        [width - 1, height - 1],
        [0, height - 1]
    ], dtype=np.float32)

    H = cv2.getPerspectiveTransform(src_points, dst_points)
    aligned_img = cv2.warpPerspective(image, H, (width, height))
    return aligned_img


def create_side_by_side(img1, img2, separator_width=32, bg_color=(255, 255, 255)):
    if img1.shape[0] != img2.shape[0]:
        height = min(img1.shape[0], img2.shape[0])
        img1 = cv2.resize(img1, (int(img1.shape[1] * height / img1.shape[0]), height))
        img2 = cv2.resize(img2, (int(img2.shape[1] * height / img2.shape[0]), height))

    separator = np.full((img1.shape[0], separator_width, 3), bg_color, dtype=np.uint8)
    return np.hstack((img1, separator, img2))


NODES_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.dirname(NODES_DIR)
IMAGES_DIR = os.path.join(PARENT_DIR, 'images')
os.makedirs(IMAGES_DIR, exist_ok=True)

def save_image(image, filename):
    path = os.path.join(IMAGES_DIR, filename)
    success = cv2.imwrite(path, image)
    if success:
        print(f"Изображение сохранено: {path}")
    else:
        print(f"Ошибка сохранения: {path}")
    return path

def show_and_save_results(original, aligned):
    combined = create_side_by_side(original, aligned)
    
    save_image(original, "captured_frame.png")
    save_image(aligned, "aligned_document.png")
    save_image(combined, "doc-homography.png")
    
    cv2.imshow("Original vs Aligned", combined)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        scanner = DocumentScanner()
        scanner.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()