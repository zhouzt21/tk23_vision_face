from tinker_vision_msgs.msg import FaceResult, Face
from tinker_vision_msgs.srv import FaceRegister
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer,Subscriber
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup,ReentrantCallbackGroup

from tinker_vision_msgs.msg import FaceResult, Face
from tinker_vision_msgs.srv import FaceRegister
from geometry_msgs.msg import Point
from std_msgs.msg import Header

from .engine import *

#### activation ####
APPID = b'8sZDPaj7x9ByVbFBHsCiCAFcD33K7FHspTRZgm3QMuoY'
SDKKey = b'Fc1z7Tb9QpTtSYkuAtpvrMV8mfURQPRygU74vMVx8y8z'

#激活接口,首次需联网激活
res = ASFOnlineActivation(APPID, SDKKey)
if (MOK != res and MERR_ASF_ALREADY_ACTIVATED != res):
    print("ASFActivation fail: {}".format(res))
else:
    print("ASFActivation sucess: {}".format(res))
# 获取激活文件信息
res,activeFileInfo = ASFGetActiveFileInfo()
if (res != MOK):
    print("ASFGetActiveFileInfo fail: {}".format(res))
else:
    print(activeFileInfo)
#### activation ####

class ReceptionistFace(Node):

    def __init__(self):
        super().__init__('receptionist_face')

        # config
        # # kinect
        # self.rgb_topic = '/rgb/image_raw'
        # self.dep_topic = '/depth_to_rgb/image_raw'
        # self.caminfo_topic = '/rgb/camera_info'
        # realsense
        self.rgb_topic = '/color/image_raw'
        self.dep_topic = '/depth/image_rect_raw'
        self.caminfo_topic = '/color/camera_info'
        ## service and topic
        self.register_service_name = '/vision/face/register'  
        # self.result_topic = '/vision/face/result' 
        # self.result_image_topic = '/vision/face/result_image'

        self.threshold = 0.85   #for registeration
        self.timeout = 3
        self.cfg_pt_min = 50

        self.rgb = Image()   # temporarily stored for registeration
        self.camera_info=CameraInfo()   

        self.state=-1  # 0: face registeration; 1: face recognition

        # data library
        self.lib_features = {}    # face library
        self.lib_info = {}        # information library
        self.count = 0  # count for registered faces

        #create register service
        self.register_service = self.create_service(
            FaceRegister, 
            self.register_service_name, 
            self.execute_cb
            # qos_profile=10
        )
        # ## create publisher
        # self.res_pub = self.create_publisher(
        #     FaceResult, 
        #     self.result_topic,
        #     qos_profile=3
        # )
        # self.res_img_pub = self.create_publisher( 
        #     Image,
        #     self.result_image_topic, 
        #     qos_profile=3
        # )  
        
        # sync RGB-D image  || create subscription
        self.img_rgb_sub =  Subscriber(
            self,
            Image, 
            self.rgb_topic 
        )
        self.img_dep_sub = Subscriber(
            self,
            Image, 
            self.dep_topic
        )
        self.img_sync=ApproximateTimeSynchronizer(
            [self.img_rgb_sub, self.img_dep_sub], 
            queue_size=10, slop=0.1
        )

        #### engine initialization #### 
        self.face_engine = ArcFace()
        mask = ASF_FACE_DETECT | ASF_FACERECOGNITION | ASF_AGE | ASF_GENDER |ASF_FACE3DANGLE | ASF_LIVENESS | ASF_IR_LIVENESS
        res = self.face_engine.ASFInitEngine(ASF_DETECT_MODE_IMAGE,ASF_OP_0_ONLY,30,10,mask)
        if (res != MOK):
            print("ASFInitEngine fail")
        else:
            print("ASFInitEngine sucess")
        #### engine initialization ####

        ## check out camera info ##
        camera_info = self.create_subscription(
            CameraInfo,
            self.caminfo_topic, 
            self.camera_callback, 
            qos_profile=10
        )    
        while not camera_info:
            print("Can't receive camera info.")
            camera_info = self.create_subscription(
                CameraInfo,
                self.caminfo_topic, 
                self.camera_callback, 
                qos_profile=10
        )
        ## check out camera info ##

        self.bridge=CvBridge()
        self.img_sync.registerCallback(self.img_cb)  ## for face recognition
        print("Face task server initialized.")

    def camera_callback(self, camera_msg):
        self.camera_info = camera_msg
        # print("camera_info received.")

    def facesToFace(self, faces, idx, face):
        face.faceRect.left = faces.faceRect[idx].left
        face.faceRect.top = faces.faceRect[idx].top
        face.faceRect.right = faces.faceRect[idx].right
        face.faceRect.bottom = faces.faceRect[idx].bottom
        face.faceOrient = faces.faceOrient[idx]
        # face.faceDataInfo = faces.faceDataInfoList[idx] 

    # def img_cb_fail(self, res_msg, fail_log):
    #     # print("img_cb_fail: "+fail_log)
    #     res_msg.success = False
    #     res_msg.fail_log = fail_log

    def img_cb(self, rgb_msg, dep_msg):    
        rgb_ori = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        dep = self.bridge.imgmsg_to_cv2(dep_msg, "32FC1")
        self.rgb=rgb_ori

    # for service response when failed
    def execute_cb_fail(self, response, fail_log): 
        print("execute_cb:"+ fail_log)
        response.success = False
        response.fail_log = fail_log
        response.id = -1
        response.rec_info = []

    def execute_cb(self, request, response):   

        if request.state == 0:  
        # if True:  # for test
            self.state=0
            print("execute_cb: Face register executing ...")

            rgb = self.rgb 
            # rgb= cv2.imread("/home/zzt/me_cropped.jpg")
    
            response = FaceRegister.Response()
            ok, faces = self.face_engine.ASFDetectFaces(rgb)
            if ok != MOK:
                self.execute_cb_fail(response, "ASFDetectFaces failed: " + str(ok))
                self.state = -1
                return response
    
            if faces.faceNum == 0:
                self.execute_cb_fail(response, "Cannot detect any faces.")
                self.state = -1
                return response
    
            if faces.faceNum >= 2:
                self.execute_cb_fail(response, "Detect more than one face.")
                self.state = -1    
                return response
    
            face = ASF_SingleFaceInfo()
            self.facesToFace(faces, 0, face)
            ok, feature = self.face_engine.ASFFaceFeatureExtract(rgb, face)
            if ok != MOK:
                self.execute_cb_fail(response, "Face feature extract fail: " + str(ok))
                self.state = -1    
                return response
    
            for i in range(len(self.lib_features)):
                ok,confidence_level = self.face_engine.ASFFaceFeatureCompare(self.lib_features[i], feature)
                if ok != MOK:
                    self.execute_cb_fail(response, "Face feature compare fail: " + str(ok))
                    self.state = -1    
                    return response
                if confidence_level > self.threshold:
                    self.execute_cb_fail(response, "Face already exists with: " + str(i))
                    self.state = -1    
                    return response
            
            mask = ASF_GENDER | ASF_AGE
            res = self.face_engine.ASFProcess(rgb, faces, mask)
            if res != MOK:
                self.execute_cb_fail(response, "ASFProcess failed:"+str(res))
                self.state = -1    
                return
            
            idx = self.count   # new feature id 
            self.count = self.count + 1  

            res, age_info = self.face_engine.ASFGetAge()
            age_info = str(age_info.ageArray[0])
            print("age_info:"+age_info)
            res, gender_info = self.face_engine.ASFGetGender()   
            gender_info = str(gender_info.genderArray[0])
            print("gender_info:"+gender_info)

            #record new person's info into data library
            info_new = [age_info, gender_info]   
            self.lib_info[idx]=info_new   # not neccessary                     
            self.lib_features[idx]=feature  
    
            #response for the service (notice: id is int type)
            response.success = True
            response.id = [idx]
            print(response.id[0])
            response.fail_log="Face register succeeded."
            response.rec_info = info_new 

            self.state = -1    
            print("Face register succeeded.")   
            return response
        elif request.state == 1:    
            print("execute_cb: Face recognition executing ...")
            # send rec_info[] when registration not when recognition, only send id[] when recognition
            self.state =1 
            rgb = self.rgb

            faces = None
            res,faces = self.face_engine.ASFDetectFaces(rgb)
            if res != MOK:
                self.execute_cb_fail(response, "ASFDetectFaces failed."+str(res))
                self.state = -1    
                return
            if faces.faceNum == 0:
                self.execute_cb_fail(response, "Cannot detect any faces."+str(res))
                self.state = -1    
                return

            response.id = []
            for i in range(faces.faceNum):    
                face = ASF_SingleFaceInfo()
                self.facesToFace(faces, i, face) 
                res, feature1 = self.face_engine.ASFFaceFeatureExtract(rgb, face)
                if res != MOK:
                    self.execute_cb_fail(response, "One possible face's feature extract fail: " + str(ok))
                    continue     
                max_conf = 0.5   # initial confidence level for face recognition
                max_id = -1  # initial id for the highest confidence level

                for key, value in self.lib_features.items():
                    res, confidence_level = self.face_engine.ASFFaceFeatureCompare(value, feature1)

                    if res != MOK:
                        print("One Face feature compare fail:"+str(res))
                        continue 
                    elif confidence_level > max_conf:
                        max_conf = confidence_level   
                        max_id = key  # update the id with the highest confidence level
                        print("Face feature compare successful with: "+str(max_id)+ ", confidence_level:"+str(max_conf))

                if max_id != -1:
                    response.id.append(max_id) # result for face recognition
                    response.success = True
                    response.fail_log = "Success with No."+str(i) 
            
            if len(response.id) == 0:
                self.execute_cb_fail(response, "No registered face is similar with input."+str(res))
                self.state = -1    
                return 
            else:
                print("Face recognition succeeded.")
                self.state = -1    
                return response

        else:
            print("execute_cb: wrong request command" )
            return
                 
            ###  extension: publish recognition result topic for visualization  ###     not work yet

            # res_msg = FaceResult()
            # res_msg.header.stamp = rgb_msg.header.stamp
            # # res_msg.header.frame_id = "kinect_rgb_camera_link"
            # res_msg.header.frame_id = "realsense_rgb_camera_link"
            # res_msg.success = True
            # res_msg.faces = []
    
                # rect = (face.faceRect.left, face.faceRect.top, face.faceRect.right - face.faceRect.left, face.faceRect.bottom - face.faceRect.top)
                # cv2.rectangle(show, rect, (200, 200, 130), 2)
                # rect2 = (face.faceRect.right, face.faceRect.top, 200, 100)
                # cv2.rectangle(show, rect2, (200, 200, 130), -1)
                # cv2.putText(show, "Id: " + str(idx + 1), (faces.faceRect[i].right, faces.faceRect[i].top + 20), cv2.FONT_HERSHEY_COMPLEX, 0.8, (255, 255, 255), 1)
                # cv2.putText(show, "Age: " + str(age_info.ageArray[i]), (faces.faceRect[i].right, faces.faceRect[i].top + 40), cv2.FONT_HERSHEY_COMPLEX, 0.8, (255, 255, 255), 1)
                # cv2.putText(show, "Gender: " + str(gender_info.genderArray[i]), (faces.faceRect[i].right, faces.faceRect[i].top + 60), cv2.FONT_HERSHEY_COMPLEX, 0.8, (255, 255, 255), 1)
    
                ##record result
                # face_msg = Face()
                # face_msg.box.xmin = face.faceRect.left
                # face_msg.box.xmax = face.faceRect.right
                # face_msg.box.ymin = face.faceRect.top
                # face_msg.box.ymax = face.faceRect.bottom
                # face_msg.age = age_info.ageArray[i]
                # face_msg.gender = gender_info.genderArray[i]
                # face_msg.id = idx + 1    
                # face_msg.info = self.lib_info[idx]
                # self.getCentroid(face_msg.box, dep, face_msg.centroid)   ##
                # self.res_face_pub.publish(face_msg)
    
                # res_msg.faces.append(face_msg)

            # msg = self.bridge.cv2_to_imgmsg(show, "bgr8")
            # self.res_img_pub.publish(msg)
            # self.res_pub.publish(res_msg)


    
    def getCentroid(self, bbox, dep_img, res):
        fx = 1.0 / self.camera_info.k[0]
        fy = 1.0 / self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        dep_img = np.array(dep_img, dtype=np.float)
        depth = []
        for v in range(bbox.ymin, bbox.ymax + 1):
            for u in range(bbox.xmin, bbox.xmax + 1):
                z = dep_img[v, u]
                if z != 0:
                    depth.append(z)
        if len(depth) < self.cfg_pt_min:
            return False

        depth.sort()
        idx = len(depth) // 10

        print("idx:"+str(idx))  ##
        res.z = depth[idx]
        res.x = res.z * (float(bbox.xmin + bbox.xmax) / 2 - cx) * fx
        res.y = res.z * (float(bbox.ymin + bbox.ymax) / 2 - cy) * fy
        return True


def main():
    rclpy.init()
    receptionist_face = ReceptionistFace()
    ## using "ros2 service call /vision/face/register tinker_vision_msgs/FaceRegister "{state: 0}" to register a face
    rclpy.spin(receptionist_face)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
