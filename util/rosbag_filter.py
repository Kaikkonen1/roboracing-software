import rosbag
import rospy
import argparse


def main():
    parser = argparse.ArgumentParser(description='Filter rosbag files.')
    parser.add_argument("--in-bag", type=str, required=True, help="Bag file this is in")
    parser.add_argument("--start-time", type=long, required=False, default=0, help="Excludes messages before this time")
    parser.add_argument("--end-time", type=long, required=False, default=None, help="Excludes messages after this time")
    parser.add_argument("--exclude-estop", type=bool, required=False, default=False, help="Excludes messages if in estop mode")
    parser.add_argument("--only-moving", type=bool, required=False, default=False, help="Excludes messages if chassis is not moving")
    parser.add_argument("--compress", type=bool, required=False, default=False, help="Creates a compressed archive of a bag file")

    args = parser.parse_args()

    file_path = args.in_bag
    start_time = args.start_time
    end_time = args.end_time
    compression = args.compress
    only_moving = args.only_moving
    exclude_estop = args.exclude_estop

    bag = rosbag.Bag(args.in_bag, 'r')
    filtered_bag = rosbag.Bag(file_path[0:file_path.rfind('.bag')] + '_filter.bag', 'w')

    if end_time is None:
        end_time = bag.get_end_time() - bag.get_start_time()

    print(bag)

    for topic, message, t in bag.read_messages(topics=['/chassis_state']):
        if not (message.header.stamp.secs <= start_time + bag.get_start_time()
                or message.header.stamp.secs >= end_time + bag.get_start_time()
                or message.estop_on == False and exclude_estop
                or message.speed_mps != 0 and only_moving):


    for writeTopic, writeMessage, writeTime in bag.read_messages():
        filtered_bag.write(topic=writeTopic, msg=writeMessage, t=writeTime)

    bag.close()
    filtered_bag.close()


if __name__ == "__main__":
    main()
