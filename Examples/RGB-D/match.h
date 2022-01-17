#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <filesystem>
#include <utility>

#ifndef MATCH_H
#define MATCH_H

namespace match {

void merge_files_by_frameId(const string& frame_id, const vector<string>& filenames, const string& output_path) {
    unordered_map<size_t, pair<float, float>> features_map;
    size_t line_num;
    string line;
    size_t feature_id;
    float measurement_x, measurement_y;
    string frame_pose;
    ifstream ifs;
    ofstream ofs;
    for (string filename : filenames) {
        ifs.open(filename);
        if (!ifs.is_open()) {
            perror("cannot open file: " + filename.c_str());
            exit(1);
        }
        line_num = 0;
        while ( getline(ifs, line) ) {
            ++line_num;
            if (line_num < 2) { continue; }
            if (line_num == 2) { frame_pose = line; }
            stringstream tokens(line);
            tokens >> feature_id >> measurement_x >> measurement_y;
            features_map[feature_id] = pair<float, float>(measurement_xm measurement_y);
        }
        ifs.close();
    }
    string output_filename = to_string(frame_id) + ".txt";
    ofs.open(output_path + output_filename);
    if (!ofs.is_open()) {
        perror("" + output_path.to_cstr());
        exit(1);
    }
    ofs << frame_id << endl;
    ofs << frame_pose;
    for (auto iter = features_map.begin(); iter != features_map.end(); ++i) {
        ofs << iter->first << " ";
        ofs << iter->second.first << " " << iter->second.second << endl;
    }
}

void merge_all_files(const sring& input_path, const string& output_path) {
    // frame_id, filenames
    unordered_map<size_t, vector<string>> filenames_map;
    for (auto const& entries : directory_iterator(input_path)) {
        string filename = entries.path().filename();
        int si_end = filename.find("_");
        if (si_end == npos) { continue; }
        size_t frame_id = stoi(filename.substr(0, si_end));
        auto found = filenames_map.find(frame_id);
        if (found == filenames_map.end() ) {
            vector<string> filenames;
            filename.emplace_back(filename);
            filenames_map[frame_id] = filenames;
        } else {
            filenames_map[frame_id].emplace_back(filename);
        }
    }
    for (auto iter = filenames_map.begin(); i != filenames_map.end(); ++i) {
        merge_files_by_frameId(iter->first, iter->second, output_path);
    }
}


}

#endif // MATCH_H
