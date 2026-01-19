# import pyzed.sl as sl
import numpy as np
from PIL import Image
import io
from sensors.ftp_server import ftp_server_login, create_dir_and_switch, upload_img_from_buffer
from dataconnections import FTPconnection

## don't use this module.
# We discontinue using the ZED Mini camera

class ZED:
    def __init__(self):
        self.zed = None
        self.ftp = None

        # Set configuration parameters
        # self.init_params = sl.InitParameters()
        # self.init_params.camera_resolution = sl.RESOLUTION.HD2K  # Set resolution
        # self.init_params.camera_fps = 15  # Set FPS
        # self.init_params.depth_mode = sl.DEPTH_MODE.NONE  # Disable depth processing for raw stereo capture

    def connect_camera(self):
        # self._check_camera_connection()
        pass

    def check_and_switch_dir(self, study_code, exp_code):
        # self.ftp = ftp_server_login()
        # create_dir_and_switch(self.ftp, study_code, exp_code)
        pass

    def take_scan(self, name):
        # # Capture stereo images
        # left_image = self._capture_stereo_image()

        # # Save images to buffer
        # left_buffer = self._save_image_to_buffer(left_image)    
        # upload_img_from_buffer(self.ftp, left_buffer, f"{name}_left.jpg")

        # # right_buffer = self._save_image_to_buffer(right_image)
        # # upload_img_from_buffer(self.ftp, right_buffer, f"{name}_right.jpg")
        pass

    def disconnect(self):
        # self.ftp.quit()
        # self.zed.close()
        pass

    
    def _capture_stereo_image(self):
        # # # Open the camera
        # # if self.zed.open(self.init_params) != sl.ERROR_CODE.SUCCESS:
        # #     raise Exception("Failed to open ZED camera.")

        # # Create image containers
        # left_image, right_image = sl.Mat(), sl.Mat()

        # # Capture frame
        # if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
        #     # Retrieve left and right images
        #     self.zed.retrieve_image(left_image, sl.VIEW.LEFT)
        #     # self.zed.retrieve_image(right_image, sl.VIEW.RIGHT)

        #     # Convert images to OpenCV format
        #     left_image = left_image.get_data()
        #     # right_image = right_image.get_data()

        #     # Convert to RGB
        #     left_image = np.array(left_image[:, :, :3])
        #     # right_image = np.array(right_image[:, :, :3])
        # else:
        #     raise Exception("Failed to capture stereo images.")

        # # self.zed.close()

        # return left_image
        pass

    def _save_image_to_buffer(self, np_image, format="JPEG"):
        """Converts a NumPy image array to an in-memory byte buffer using PIL."""
        # Convert NumPy array to a PIL Image
        # image = Image.fromarray(np_image)

        # # Create an in-memory buffer
        # img_buffer = io.BytesIO()

        # # Save the image to the buffer
        # image.save(img_buffer, format=format)

        # # Move cursor to the start of the buffer
        # img_buffer.seek(0)

        # return img_buffer  # Now this buffer can be used directly for FTP or other operations
        pass


    def _check_camera_connection(self):
        # zed = sl.Camera()
        # status = zed.open(self.init_params)
        
        # if status == sl.ERROR_CODE.SUCCESS:
        #     print("✅ ZED Mini connected successfully and ready for scans!")
        #     self.zed = zed
        # else:
        #     raise Exception("❌ ZED Mini not detected. Check USB connection.")
        pass
        
