import sys
import bag_tagger  # in local folder

params = {
    "classes_of_interest": ["cassowary"],
    "rgb_cam_threshold": 0.6,
    "thermal_threshold": 0.6,
    "count_threshold": 5,
}

for line in sys.stdin:
    # print('-------------')
    bag_path = line.rstrip()
    # print(bag_path)
    if bag_tagger.bag_tagger(bag_path, params):
        print(bag_path)
