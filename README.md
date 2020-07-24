bag2video
=========

Convert images from multiple topics in a rosbag to a constant framerate video with topics displayed side to side. Conversion from timestamps to constant framerate is achieved through duplicating frames. Images for each topic will be scaled to the same height and placed side to side horizontally.

This should not be used for precise conversions. The primary purpose is to provide a visual for the image contents of a bag file for multiple topics at once. There are several quirks present as a tradeoff for simplicity and faster processing:

* The program updates for every relevant message in the bag file. Each update, the most recently processed images from each topic are horizontally concatenated. The resulting image is written for duration equal to the time since the last update. Black images are used as substitutes for topics that have not had a message.
* If a particular topic ends earlier than the rest, then the program will continue using the most recent image from that topic. This behavior may or may not be desirable.
* Because bag files use timestamps, there is no information on how long the last message should last. The program avoids this problem for the last output image by not writing it at all.

This script is heavily modified from the original; it uses Python 3.

# Usage
    usage: bag2video.py [-h] [--index INDEX] [--scale SCALE] [--outfile OUTFILE] [--fps FPS]
                        [--viz] [--start START] [--end END] [--encoding {rgb8,bgr8,mono8}]
                        [--fourcc FOURCC] [--log LOG]
                        bagfile topics [topics ...]

    Extract and encode video from bag files.

    positional arguments:
      bagfile               Specifies the location of the bag file.
      topics                Image topics to merge in output video.

    optional arguments:
      -h, --help            show this help message and exit
      --index INDEX, -i INDEX
                            Resizes all images to match the height of the topic specified.
                            Default 0.
      --scale SCALE, -x SCALE
                            Global scale for all images. Default 1.
      --outfile OUTFILE, -o OUTFILE
                            Destination of the video file. Defaults to the folder of the bag
                            file.
      --fps FPS, -f FPS     FPS of the output video. If not specified, FPS will be set to the
                            maximum frequency of the topics.
      --viz, -v             Display frames in a GUI window.
      --start START, -s START
                            Rostime representing where to start in the bag.
      --end END, -e END     Rostime representing where to stop in the bag.
      --encoding {rgb8,bgr8,mono8}
                            Encoding of the deserialized image. Default bgr8.
      --fourcc FOURCC, -c FOURCC
                            Specifies FourCC for the output video. Default MJPG.
      --log LOG, -l LOG     Logging level. Default INFO.

## Steps to setting up a ROS + Python3 environment in conda

```
conda create --name rosbag python=3.7
conda activate rosbag
conda install numpy pycryptodome imageio opencv
export PYTHONPATH=~/miniconda3/envs/rosbag/lib/python3.7/site-packages
export LD_LIBRARY_PATH=~/miniconda3/envs/rosbag/lib/
pip install --extra-index-url https://rospypi.github.io/simple/ rospy rosbag roslz4 imageio-ffmpeg cv_bridge sensor_msgs geometry_msgs
```
