# tk23_vision_face
## face_recongition_arcface

 Using Arcsoft SDK to complete face recongition task.

 1. First use Kinect or Realsense Camera for the image input, and chose the correct subscribed image topic name according to the camera you use. ```class ReceptionistFace()``` is defined in ```
 /face_recognition_arcface/face_recognition_arcface/receptionist_face_task.py```, and the topic name configuration is in it.

 2. (*) [Arcsoft face SDK](https://ai.arcsoft.com.cn/product/arcface.html) : Register an account and get APPID and SDKKEY for face recognition SDK, and change APPID and SDKKEY in /face_recognition_arcface/receptionist_face_task.py.

 3. (*) Test the SDK demo: Copy folder /lib altogether (with two .so file right in it) in SDK you download, put under /face_recognition_arcface/arcface_demo, then run this to test: 

```python 
python demo.py
```

 4. Run the face recognition node:
    
```shell
ros2 run face_recognition_arcface receptionist_face_task.py
```

 5. Call the face registeration service:

```shell
ros2 service call /vision/face/register tinker_vision_msgs/FaceRegister "{state: 0}"
```
Call the face comparation service:
```shell
ros2 service call /vision/face/register tinker_vision_msgs/FaceRegister "{state: 1}"
```
