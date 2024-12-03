# Steps to install
1. Set up git on your computer (find a tutorial)
2. Set up ssh keys so that you can clone the repo (find a tutorial)
3. `git clone https://www.github.com/rahul-io/cv-project-f24.git`
4. `cd cv-project-f24`
5. `docker build -t cv-project .`
6. `docker run -it cv-project`
7. `docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $HOME/.Xauthority:/root/.Xauthority -e XAUTHORITY=/root/.Xauthority -v <any-other-local-folders>:<where-to-mount-directory> cv-project`


# To run yolo
1. From cv-project-f24 folder
2. `cd Yolo/yolov7`
3. `python3 ros_detect.py --weights runs/train/yolo_slam6/weights/best.pt --conf-thres 0.7 --img-size 640`
4. conf-thresh - confidence threshold, img-size - image size as a square, 
