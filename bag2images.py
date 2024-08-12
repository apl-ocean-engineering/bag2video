#!/usr/bin/env python3

from __future__ import division
import rosbag, rospy, numpy as np
import sys, os, cv2, glob
from itertools import repeat
import imageio
import argparse
import logging
import traceback
from pathlib import Path
from cv_bridge import CvBridge


def get_sizes(bag, topics=None, index=0, scale=1.0, start_time=rospy.Time(0), stop_time=rospy.Time(sys.maxsize)):
    logging.debug("Resizing height to topic %s (index %d)." % (topics[index] , index))
    sizes = []

    for topic in topics:
        try:
            iterator = bag.read_messages(topics=topic, start_time=start_time, end_time=stop_time)#, raw=True)
            msg = next(iterator)[1] # read one message
            sizes.append((msg.width, msg.height))
        except:
            logging.critical("No messages found for topic %s, or message does not have height/width." % topic)
            traceback.print_exc()
            sys.exit(1)

    target_height = int(sizes[index][1]*scale)

    # output original and scaled sizes
    for i in range(len(topics)):
        logging.info('Topic %s originally of height %s and width %s' % (topics[i],sizes[i][0],sizes[i][1]))
        image_height = sizes[i][1]
        image_width = sizes[i][0]

        # rescale to desired height while keeping aspect ratio
        sizes[i] = (int(1.0*image_width*target_height/image_height),target_height)
        logging.info('Topic %s rescaled to height %s and width %s.' % (topics[i],sizes[i][0],sizes[i][1]))

    return sizes

def calc_out_size(sizes):
    return (sum(size[0] for size in sizes),sizes[0][1])

def merge_images(images, sizes):
    return cv2.hconcat([cv2.resize(images[i],sizes[i]) for i in range(len(images))])

def write_frames(bag, outdir, topics, sizes, start_time=rospy.Time(0),
                    stop_time=rospy.Time(sys.maxsize), viz=False, encoding='bgr8', skip=1):
    bridge = CvBridge()
    convert = { topics[i]:i for i in range(len(topics))}
    images = [np.zeros((sizes[i][1],sizes[i][0],3), np.uint8) for i in range(len(topics))]
    count = 0

    iterator = bag.read_messages(topics=topics, start_time=start_time, end_time=stop_time)

    for topic, msg, t in iterator:
        time=t.to_sec()
        logging.debug('Topic %s updated at time %s seconds' % (topic, time ))

        image = np.asarray(bridge.imgmsg_to_cv2(msg, encoding))
        images[convert[topic]] = image

        if count % skip == 0:
            logging.info('Writing image %s at time %.6f seconds.' % (count, time) )
            outpath = outdir / ( "image_%06d.png" % count )
            logging.debug("Writing %s" % outpath)
            merged_image = merge_images(images, sizes)
            imageio.imwrite( outpath, merged_image )

        count += 1
    # Write the last frame if it was skipped in the last loop
    if count % skip != 0:
        outpath = outdir / ("image_%06d.png" % count)
        logging.debug("Writing %s" % outpath)
        merged_image = merge_images(images, sizes)
        imageio.imwrite(outpath, merged_image)

def create_thumbnail(bag, outdir, topics, sizes, start_time=rospy.Time(0),
                             stop_time=rospy.Time(sys.maxsize), encoding='bgr8', collage_size=(4,4)):
    bridge = CvBridge()
    convert = { topics[i]:i for i in range(len(topics))}
    selected_images = []

    iterator = bag.read_messages(topics=topics, start_time=start_time, end_time=stop_time)
    total_frames = sum(1 for _ in bag.read_messages(topics=topics, start_time=start_time, end_time=stop_time))

    step = max(1, total_frames // (collage_size[0] * collage_size[1]))

    for i, (topic, msg, t) in enumerate(iterator):
        if i % step == 0:
            image = np.asarray(bridge.imgmsg_to_cv2(msg, encoding))
            selected_images.append(image)
            if len(selected_images) >= collage_size[0] * collage_size[1]:
                break

    if len(selected_images) < collage_size[0] * collage_size[1]:
        logging.warning("Not enough images to fill the entire collage. The collage will contain empty spaces.")
        selected_images += [np.zeros_like(selected_images[0])] * (collage_size[0] * collage_size[1] - len(selected_images))

    # Resize images and create collage
    collage_images = [cv2.resize(img, (sizes[0][0], sizes[0][1])) for img in selected_images]
    rows = [cv2.hconcat(collage_images[i*collage_size[1]:(i+1)*collage_size[1]]) for i in range(collage_size[0])]
    collage = cv2.vconcat(rows)

    # Save the collage
    thumbnail_path = outdir / "thumbnail_collage.png"
    logging.info(f'Writing thumbnail collage to {thumbnail_path}')
    cv2.imwrite(str(thumbnail_path), collage)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Extract and encode images from bag files.')
    parser.add_argument('bagfile', help='Specifies the location of the bag file.')
    parser.add_argument('topics', nargs='+',help='Image topics to merge in output directory.')
    parser.add_argument('--index', '-i', action='store',default=0, type=int,
                        help='Resizes all images to match the height of the topic specified. Default 0.')
    parser.add_argument('--scale', '-x', action='store',default=1, type=float,
                        help='Global scale for all images. Default 1.')
    parser.add_argument('--outdir', '-o', action='store', required=True, type=Path,
                        help='Destination directory for output')

    parser.add_argument('--start', '-s', action='store', default=0, type=float,
                        help='Rostime representing where to start in the bag.')
    parser.add_argument('--end', '-e', action='store', default=sys.maxsize, type=float,
                        help='Rostime representing where to stop in the bag.')

    parser.add_argument('--skip', default=1, type=int,
                        help='Extract every N\'th image.')


    parser.add_argument('--encoding', choices=('rgb8', 'bgr8', 'mono8'), default='bgr8',
                        help='Encoding of the deserialized image. Default bgr8.')

    parser.add_argument('--log', '-l',action='store',default='INFO',
                        help='Logging level. Default INFO.')
    
    parser.add_argument('--thumbnail', default=False, type=bool,
                       help='Generate thumbnail? Default False.')

    args = parser.parse_args()

    # logging setup
    numeric_level = getattr(logging, args.log.upper(), None)
    if not isinstance(numeric_level, int):
        raise ValueError('Invalid log level: %s' % loglevel)
    logging.basicConfig(format='%(asctime)s - %(levelname)s - %(message)s',level=numeric_level)
    logging.info('Logging at level %s.',args.log.upper())

    args.outdir.mkdir(exist_ok=True)

    # convert numbers into rospy Time
    start_time=rospy.Time(args.start)
    stop_time=rospy.Time(args.end)

    try:
        assert start_time <= stop_time
    except:
        logging.critical("Start time is after stop time.")
        traceback.print_exc()
        sys.exit(1)

    try:
        assert args.index < len(args.topics)
    except:
        logging.critical("Index specified for resizing is out of bounds.")
        traceback.print_exc()
        sys.exit(1)

    for bagfile in glob.glob(args.bagfile):
        logging.info('Proccessing bag %s.'% bagfile)
        bag = rosbag.Bag(bagfile, 'r')

        logging.info('Calculating image sizes.')
        sizes = get_sizes(bag, topics=args.topics, index=args.index,scale = args.scale, start_time=start_time, stop_time=stop_time)

        logging.info('Calculating final image size.')
        out_width, out_height = calc_out_size(sizes)
        logging.info('Resulting images of width %s and height %s.'%(out_width,out_height))

        if not args.thumbnail:
            write_frames(bag=bag, outdir=args.outdir, topics=args.topics, sizes=sizes,
                         start_time=start_time, stop_time=stop_time, encoding=args.encoding, skip=args.skip)
        else: 
            create_thumbnail(bag=bag, outdir=args.outdir, topics=args.topics, sizes=sizes,
                         start_time=start_time, stop_time=stop_time, encoding=args.encoding)
        

        logging.info('Done.')
