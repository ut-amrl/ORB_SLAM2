import os
import sys, getopt
from collections import defaultdict


def add_to_features_mono(features_dict, line):
    tokens = line.split()
    feature_id = int(tokens[0])
    # ss = line[len(tokens[0]):]
    # features_dict[feature_id] = ss 
    measurement_x = float(tokens[1])
    measurement_y = float(tokens[2])
    features_dict[feature_id] = (measurement_x, measurement_y)

def add_to_features_stereo(features_dict, line):
    tokens = line.split()
    feature_id = int(tokens[0])
    measurement_x1 = float(tokens[1])
    measurement_y1 = float(tokens[2])
    depth = float(tokens[3])
    if (depth < 0): # TODO: need to figure out why it happens
        return
    measurement_x2 = float(tokens[4])
    measurement_y2 = float(tokens[5])
    features_dict[feature_id] = (depth, measurement_x1, measurement_y1, measurement_x2, measurement_y2)

def add_to_features(features_dict, line, stereo=True):
    if stereo:
        add_to_features_stereo(features_dict, line)
    else:
        add_to_features_mono(features_dict, line)

def associate_odom_TUM(dataset_path, timestamp):
    fp = open(dataset_path + "groundtruth.txt")
    lines = fp.readlines()
    for line in lines:
        if line[0] == "#":
            continue # skip comments
        tokens = line.split()
        time = float(tokens[0])
        if time > timestamp:
            odom = ""
            for token in tokens[1:]:
                odom = odom + token + " "
            odom = odom + "\n"
            return time,odom
    return None

def associate_odom(dataset_path, timestamp, which_dataset="TUM"):
    return associate_odom_TUM(dataset_path, timestamp)

def merge_files(fps_list, fp_out, fp_depth_out=None, dataset_path=None, timestamp=None):
    lines_list = []
    for fp in fps_list:
        lines_list.append(fp.readlines())
    frame_id = ""
    frame_pose = ""
    features_dict = dict()
    for lines in lines_list:
        frame_id = lines[0]
        frame_pose = lines[1]
        for line in lines[2:]:
            add_to_features(features_dict, line)
    fp_out.write(frame_id)
    if fp_depth_out is not None:
        fp_depth_out.write(frame_id)
    if timestamp is not None:
        if dataset_path == None:
            print("dataset-path is unintialized; exiting")
            exit(1)
        time, frame_pose = associate_odom(dataset_path, timestamp)
        # print(time , frame_pose)
    fp_out.write(frame_pose)
    if fp_depth_out is not None:
        fp_depth_out.write(frame_pose)
    for feature_id, measurement in features_dict.items():
        if len(measurement) == 2:
            fp_out.write(str(feature_id+1) + " " + str(measurement[0]) + " " + str(measurement[1]) + "\n")
        else:
            fp_out.write(str(feature_id+1) + " " + str(measurement[1]) + " " + str(measurement[2]) + " " + str(measurement[3]) + " " + str(measurement[4]) + "\n")
            fp_depth_out.write(str(feature_id+1) + " " + str(measurement[0]) + "\n")
    fp_out.close()
    fp_depth_out.close()

if __name__ == '__main__':
    dataset_path = None
    input_path = None
    output_path = None
    camera_type = "stereo"
    try:
        opts, args = getopt.getopt(sys.argv[1:],"hi:o:d:t:",["input-path=","output-path=", "dataset-path=", "camera-type="])
    except getopt.GetoptError:
        print('test.py -i <input-path> -o <output-path> -d <dataset-path> -t <camera-type>')
        sys.exit(1)
    for opt, arg in opts:
        if opt == '-h':
            print('test.py -i <dataset-path> -o <output-path> -d <dataset-path> -t <camera-type>')
        elif opt in ("-i", "dataset-path"):
            input_path = arg
        elif opt in ("-o","output-path"):
            output_path = arg
        elif opt in ("-d", "dataset-path"):
            dataset_path = arg
        elif opt in ("-t", "camera-type"):
            camera_type = arg
    if  output_path == None:
        print("missing dataset_path or output_path")
        exit(1)
    print("input_path", input_path)
    print("output_path", output_path)
    print("dataset_path", dataset_path)
    filenames_dict = defaultdict(list)
    for filename in os.listdir(input_path):
        tokens = filename.split("_")
        try:
            frame_id = int(tokens[0])
        except ValueError as e:
            continue
        try:
            timestamp = float(tokens[-1].split(".")[0])
        except ValueError as e:
            print("unexpected ValueError when parsing timestamp " + timestamp)
            exit(1)
        filenames_dict[(frame_id, timestamp)].append(filename)
    for (frame_id, timestamp), filenames in filenames_dict.items():
        fps_list = []
        for filename in filenames:
            fp = open(input_path + filename, "r")
            if fp.closed:
                print("cannot open file " + dataset_path + filename)
            fps_list.append(fp)
        fp_out = open(output_path + str(frame_id) + ".txt", "w")
        fp_out.seek(0)
        fp_out.truncate()
        if camera_type == "stereo":
            if not os.path.exists(output_path + "features/"):
                os.makedirs(output_path + "features/")
            fp_depth_out = open(output_path + "features/" + str(frame_id) + ".txt", "w")
            fp_depth_out.seek(0)
            fp_depth_out.truncate()
        else:
            fp_depth_out=None
        if dataset_path == None:
            merge_files(fps_list, fp_out, fp_depth_out)
        else:
            merge_files(fps_list, fp_out, fp_depth_out, dataset_path, timestamp) 


