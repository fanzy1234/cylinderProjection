from __future__ import print_function

import quaternion
import roslib
roslib.load_manifest('wom_cafe')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import threading

import matplotlib.pyplot as plt

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub0 = rospy.Subscriber("/IMX185_0/image_raw", Image, self.callback0)
    self.image_sub1 = rospy.Subscriber("/IMX185_1/image_raw", Image, self.callback1)
    self.image_sub2 = rospy.Subscriber("/IMX185_2/image_raw", Image, self.callback2)

    self.camera_info0 = rospy.Subscriber("/IMX185_0/camera_info", CameraInfo, self.camera_info_callback0)
    self.camera_info1 = rospy.Subscriber("/IMX185_1/camera_info", CameraInfo, self.camera_info_callback1)
    self.camera_info2 = rospy.Subscriber("/IMX185_2/camera_info", CameraInfo, self.camera_info_callback2)
    self.projection_matrix0 = None
    self.projection_matrix1 = None
    self.projection_matrix2 = None
    self.projection_matrix_set = False
    self.res_img = np.zeros((1080, 5760, 3), dtype=np.uint8)
    self.ready0 = False
    self.ready1 = False
    self.ready2 = False
    self.sem0 = threading.Semaphore()
    self.sem1 = threading.Semaphore()
    self.sem2 = threading.Semaphore()



  def callback0(self,data):
    try:
      self.image0 = self.bridge.imgmsg_to_cv2(data, "bgr8")
      if self.projection_matrix_set:
        for i in range(1080):
          for j in range(1920):
            coord = self.projection_matrix0[j, i]
            self.res_img[int(coord[1]), int(coord[0])] = self.image0[i, j]
        self.ready0 = True
        print("img0 is ready!")
        self.sem0.acquire()

    except CvBridgeError as e:
      print(e)

  def callback1(self,data):
    try:
      self.image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
      if self.projection_matrix_set:
        for i in range(1080):
          for j in range(1920):
            coord = self.projection_matrix1[j, i]
            self.res_img[int(coord[1]), int(coord[0])] = self.image1[i, j]
        self.ready1 = True
        print("img1 is ready!")
        self.sem1.acquire()

    except CvBridgeError as e:
      print(e)

  def callback2(self,data):
    try:
      self.image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
      if self.projection_matrix_set:
        for i in range(1080):
          for j in range(1920):
            coord = self.projection_matrix2[j, i]
            self.res_img[int(coord[1]), int(coord[0])] = self.image2[i, j]
        self.ready2 = True
        print("img2 is ready!")
        self.sem2.acquire()

    except CvBridgeError as e:
      print(e)
    
  def camera_info_callback0(self, data):
      self.intrinsic_mat0 = np.reshape(np.array(data.K), (3, 3))

  def camera_info_callback1(self, data):
      self.intrinsic_mat1 = np.reshape(np.array(data.K), (3, 3))

  def camera_info_callback2(self, data):
      self.intrinsic_mat2 = np.reshape(np.array(data.K), (3, 3))

  def set_extr_mat(self, cam_tf):

    rotation = cam_tf.transform.rotation

    translation = cam_tf.transform.translation

    quat = np.quaternion(rotation.w, rotation.x, rotation.y, rotation.z)

    self.rot_mat = quaternion.as_rotation_matrix(quat)

    self.tr_mat = np.array((translation.x, translation.y, translation.z)).reshape(3,1)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)

  # ic.projection_matrix0 = np.load('projection_matrix_0.npy')
  # ic.projection_matrix1 = np.load('projection_matrix_1.npy')
  # ic.projection_matrix2 = np.load('projection_matrix_2.npy')

  ic.projection_matrix1 = calculate_proj_mat(ic, 1)
  ic.projection_matrix0 = calculate_proj_mat(ic, 0)
  ic.projection_matrix2 = calculate_proj_mat(ic, 2)

  # plt.plot(ic.projection_matrix2[:, 0], ic.projection_matrix2[:, 1], 'bo')
  # plt.plot(ic.projection_matrix0[:, 0], ic.projection_matrix0[:, 1], 'bo')
  # plt.plot(ic.projection_matrix1[:, 0], ic.projection_matrix1[:, 1], 'bo')
  #
  # plt.show()

  ic.projection_matrix_set = True
  print("projection_matrix is set!")


  # plt.plot(p[:, 0], p[:, 1], 'bo')
  # plt.show()

  # cv2.namedWindow("Image window", cv2.WINDOW_NORMAL)
  while(True):
    if (ic.ready0 and ic.ready1 and ic.ready2):
        ic.ready0 = False
        ic.ready1 = False
        ic.ready2 = False
        cv2.imshow("Image window", ic.res_img)
        cv2.waitKey(0)
        ic.sem0.release()
        ic.sem1.release()
        ic.sem2.release()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

def calculate_proj_mat(ic, k):
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    j = 0
    while True:
        try:
            cam_tf = tfBuffer.lookup_transform('IMX185_' + str(k) + '_base_link', 'velodyne_base_link', rospy.Time())
            ic.set_extr_mat(cam_tf)
            j += 1
            if (j == 10):
                break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

    inv = np.linalg.inv(ic.intrinsic_mat0)
    rot_mat_inv = np.linalg.inv(ic.rot_mat)

    print("rot_mat_" + str(k) + ": ", ic.rot_mat)
    print("tr_mat_" + str(k) + ": ", ic.tr_mat)

    projection_matrix = np.zeros([1920, 1080, 2])

    for i in range(1920):
        for j in range(1080):
            pt = np.dot(inv, np.array([i, j, 1], dtype=np.float32))
            # print(pt)
            # pt = np.dot(rot_mat_inv, np.array([pt[2], pt[0], pt[1]])) - ic.tr_mat.T
            pt = np.array([pt[2], -pt[0], -pt[1]]) - ic.tr_mat.T
            pt = pt[0]
            pt = np.dot(rot_mat_inv, pt)
            projection_matrix[i, j, 0] = -np.arctan2(pt[1], pt[0]) * 960 + 2880
            projection_matrix[i, j, 1] = -pt[2] / (np.sqrt(pt[0] * pt[0] + pt[1] * pt[1])) * 540 + 540

            # projection_matrix[i, j, 0] = pt[0]
            # projection_matrix[i, j, 1] = pt[1]

            # print("camera: " + str(k) + " i: " + str(i) + " j: " + str(j) + " z: " + str(pt[2]))
        # print("camera: " + str(k) + " i: " + str(i) + " j: " + str(j) + " x: " + str(pt[0]) + " y: " + str(pt[1]))
        #     if (i == 0):
        #         print("i: " + str(i) + " j: " + str(j) + " x: " + str(projection_matrix[i, j, 0]) + " y: " + str(projection_matrix[i, j, 1]))

    # adfa
    p = projection_matrix.reshape(1920 * 1080, 2)
    # plt.plot(p[:, 0], p[:, 1], 'bo')
    # plt.show()
    # asdf

    print('x: {:.5f} to {:.5f}, y: {:.5f} to {:.5f}'.format(np.amin(p[:, 0]), np.amax(p[:, 0]), np.amin(p[:, 1]),
                                                            np.amax(p[:, 1])))
    projection_matrix = projection_matrix.astype(np.int32)
    projection_matrix[:, :, 0] = np.clip(projection_matrix[:, :, 0], 0, 5759)
    projection_matrix[:, :, 1] = np.clip(projection_matrix[:, :, 1], 0, 1079)
    # np.save('projection_matrix_2.npy', projection_matrix)

    # ic.projection_matrix0 = np.load('projection_matrix_0.npy')
    # ic.projection_matrix1 = np.load('projection_matrix_1.npy')
    # ic.projection_matrix2 = np.load('projection_matrix_2.npy')

    # ic.projection_matrix_set = True
    print("projection_matrix " + str(k) + " is set!")
    # return projection_matrix
    return projection_matrix


def convert_point(x, y, w, h):
    x1 = float(x - w/2)
    y1 = float(y - h/2)

    f = float(w/2)
    r = float(w)

    omega = float(w/2)
    z0 = f - np.sqrt(r*r - omega*omega)
    
    zc = (2*z0 + np.sqrt(4*z0*z0 - 4*(x1*x1/(f*f) + 1)*(z0*z0 - r*r)))/(2*(x1*x1/(f*f) + 1))
    
    x2 = float(x1*zc/f + w/2)
    y2 = float(y1*zc/f + h/2)
    return (x2, y2)

def convert_image(frame):
    new_frame = np.zeros(frame.shape, np.uint8)
    width = new_frame.shape[1]
    height = new_frame.shape[0]
    for y in range(height):
        for x in range(width):
            cur_pos_x, cur_pos_y = convert_point(float(x), float(y), width, height)
            top_left_x = int(cur_pos_x)
            top_left_y = int(cur_pos_y)

            if (top_left_x < 0 or 
                top_left_x > width - 2 or 
                top_left_y < 0 or
                top_left_y > height - 2):
                continue
                
            dx = float(cur_pos_x - top_left_x)
            dy = float(cur_pos_y - top_left_y)

            weight_tl = (1.0 - dx) * (1.0 - dy)
            weight_tr = dx * (1.0 - dy)
            weight_bl = (1.0 - dx) * dy
            weight_br = dx * dy

            val = weight_tl * frame[top_left_y, top_left_x] + weight_tr * frame[top_left_y, top_left_x + 1] + weight_bl * frame[top_left_y + 1, top_left_x] + weight_br * frame[top_left_y + 1, top_left_x + 1]
            new_frame[y, x] = val.astype(np.uint8)
    
    return new_frame

if __name__ == '__main__':
    main(sys.argv)
