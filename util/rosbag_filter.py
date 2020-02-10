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
    use_index = []

    bag = rosbag.Bag(args.in_bag, 'r')
    filtered_bag = rosbag.Bag(file_path[0:file_path.rfind('.bag')] + '_filter.bag', 'w')

    if end_time is None:
        end_time = bag.get_end_time() - bag.get_start_time()

    print(bag)

    #for topic, message, t in bag.read_messages(topics=['/chassis_state', '/imu/data_raw']):
    #    if topic == '/chassis_state':
    #        use_index.append(
    #            start_time + bag.get_start_time() <= message.header.stamp.secs <= end_time + bag.get_start_time()
    #            and not (message.estop_on == True and exclude_estop)
    #        )
    #    elif topic == '/imu/data_raw':
    #        if only_moving:
    #            use_index.append(
    #                not (abs(message.angular_velocity.x) < 5 and abs(message.angular_velocity.y) < 5 and abs(message.angular_velocity.z) < 5)
    #            )

    for topic, message, t in bag.read_messages(topics=['/chassis_state', '/imu/data_raw']):
        if topic == '/chassis_state':
            use_index.append([start_time + bag.get_start_time() <= message.header.stamp.secs <= end_time + bag.get_start_time()
                              and not (message.estop_on == True and exclude_estop), float(str(message.header.stamp.secs) + '.' + str(message.header.stamp.nsecs))])
        elif topic == '/imu/data_raw':
            use_index.append([only_moving and not (abs(message.angular_velocity.x) < 5 and abs(message.angular_velocity.y) < 5 and abs(message.angular_velocity.z) < 5),
                              float(str(message.header.stamp.secs) + '.' + str(message.header.stamp.nsecs))])

    #count = 0

    #for write_topic, write_message, write_t in bag.read_messages():
    #    if use_index[count]:
    #        filtered_bag.write(topic=write_topic, msg=write_message, t=write_t)
    #    if write_topic == "/tf":
    #        count += 1
    print use_index
    for write_topic, write_message, write_t in bag.read_messages():
        if write_topic == "/tf":
            write_message = write_message.transforms[0]
        print float(str(write_message.header.stamp.secs) + '.' + str(write_message.header.stamp.nsecs))
        if is_within_times(write_message.header.stamp, use_index):
            filtered_bag.write(topic=write_topic, msg=write_message, t=write_t)

    bag.close()
    filtered_bag.close()


def is_within_times(stamp, use_index):
    for i in range(len(use_index) - 2):
        if (use_index[i][0] or use_index[i + 1][0]) and use_index[i][1] < float(str(stamp.secs) + '.' + str(stamp.nsecs)) < use_index[i + 1][1] and (use_index[i + 1][1] - use_index[i][1]) < 0.1:
            return True
    return False

# and\ use_index[i][0] < float(str(stamp.secs) + '.' + str(stamp.nsecs)) < use_index[i + 1][0] and (use_index[i + 1][0] - use_index[i][0]) < 0.1

if __name__ == "__main__":
    main()
