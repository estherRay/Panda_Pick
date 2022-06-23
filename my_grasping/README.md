This folder is for Pick and Place aplications with Panda arm.
You need to launch one of the launch file and then its corresponding python file (ex: full_demo_test.launch and full_demo_test.py)
Some launch files use Find_object_3d and the rest use AprilTag detection.

For find_object_3d, the known objects can be find in find_object_2d/Objects. You can add new ones.
For AprilTag, the library is installed on Panda's real-time kernel.
      The tags used are from Tag16 folder. You need to have a printed tag from this folder on the object to be picked up.
      The tag bumber and the size of the tag on the object to be picked need to be specified in the AprilTag folder.

