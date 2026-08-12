/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/map_creator/tile_map_images_creator/grid_map/grid_matrix.h"

#include <fstream>
#include <cmath>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include "cyber/cyber.h"
#include "modules/map_creator/tile_map_images_creator/common/utils.h"

namespace apollo {
namespace tile_map_images_creator {

GridMatrix::GridMatrix(const GridMatrix& mapcells) {
    Init(mapcells.option_);
    for (unsigned int y = 0; y < rows_; ++y) {
        for (unsigned int x = 0; x < cols_; ++x) {
            GridCell& cell = get_mutable_map_cell(y, x);
            const GridCell& src_cell = mapcells.get_const_map_cell(y, x);
            cell = GridCell(src_cell);
        }
    }
}

GridMatrix::GridMatrix(std::shared_ptr<GridOption> option) {
    Init(option);
}

void GridMatrix::Reset() {
    map_cells_.resize(0);
    std::cerr << "free map3d matrix..." << std::endl;
}

void GridMatrix::Init(std::shared_ptr<GridOption> option) {
    unsigned int rows = option->matrix_size;
    unsigned int cols = option->matrix_size;
    float resolution = option->resolution;
    resolution_id_ = option->resolution_id;
    option_ = option;
    map_cells_.assign(rows * cols, GridCell());
    for (unsigned int i = 0; i < rows; ++i) {
        for (unsigned int j = 0; j < cols; ++j) {
            map_cells_[i * cols + j].set_index_x(i);
            map_cells_[i * cols + j].set_index_y(j);
            map_cells_[i * cols + j].set_resolution(resolution);
        }
    }
    rows_ = rows;
    cols_ = cols;
    resolution_ = resolution;
}

void GridMatrix::SetValue(
        double x,
        double y,
        double z,
        double z_relative,
        double z_relative_var,
        double z_relative_min,
        double z_relative_max,
        unsigned int count,
        double intensity,
        unsigned char r,
        unsigned char g,
        unsigned char b,
        int layer) {
    const double matrix_resolution = option_->resolution * option_->matrix_size;
    int matrix_index_x = std::floor(x / matrix_resolution);
    int matrix_index_y = std::floor(y / matrix_resolution);
    double matrix_min_x = static_cast<double>(matrix_index_x * matrix_resolution);
    double matrix_min_y = static_cast<double>(matrix_index_y * matrix_resolution);
    int row = static_cast<int>((x - matrix_min_x) / resolution_);
    int col = static_cast<int>((y - matrix_min_y) / resolution_);
    // AINFO << "GridMatrix::SetValue row=" << row << ", col=" << col;
    row = row > 0 ? row : -row;
    col = col > 0 ? col : -col;

    GridCell& map_cell = get_mutable_map_cell(row, col);
    map_cell.SetValue(z, z_relative, z_relative_var, z_relative_min, z_relative_max, count, intensity, r, g, b, layer);
}

void GridMatrix::LoadFusePoints(std::vector<GridCellSingle>& fuse_points) {
    for (unsigned int i = 0; i < rows_; ++i) {
        for (unsigned int j = 0; j < cols_; ++j) {
            GridCell& mapcell = get_mutable_map_cell(i, j);
            mapcell.GetSingleCells(fuse_points);
        }
    }
}

std::string GridMatrix::GetMatrixKey() const {
    return boost::str(
            boost::format("resolution_%d_%d_%d") % resolution_id_ % matrix_index_.index_x_ % matrix_index_.index_y_);
}

size_t GridMatrix::GetBinPointSize() const {
    return sizeof(unsigned char) * 4 + sizeof(float) * 7 + sizeof(int) * 1 + sizeof(double) * 1;
}

// 此处只保存了数值均值，并没有保存方差，若后续机器学习任务需要，仍需要补充
unsigned int GridMatrix::save_buffer(std::vector<unsigned char>& buf) {
    std::vector<GridCellSingle>& fuse_points = fuse_points_;
    if (fuse_points.size() == 0) {
        LoadFusePoints(fuse_points_);
    }
    unsigned int byte_num = 0;
    // 每组一共占用字节，如果需要增加存储项（如：增加方差），此处需要调整
    buf.resize(GetBinPointSize() * fuse_points.size());
    unsigned char* p = &buf[0];
    for (unsigned int index = 0; index < fuse_points.size(); ++index) {
        if (fuse_points[index].count_ == 0) {
            continue;
        }
        byte_num += WriteMessageAndShift(p, static_cast<unsigned char>(fuse_points[index].layer_));
        byte_num += WriteMessageAndShift(
                p, static_cast<float>(fuse_points[index].resolution_ * fuse_points[index].index_x_));
        byte_num += WriteMessageAndShift(
                p, static_cast<float>(fuse_points[index].resolution_ * fuse_points[index].index_y_));
        byte_num += WriteMessageAndShift(p, static_cast<float>(fuse_points[index].altitude_));
        byte_num += WriteMessageAndShift(p, static_cast<float>(fuse_points[index].altitude_relative_));
        byte_num += WriteMessageAndShift(p, static_cast<float>(fuse_points[index].altitude_relative_var_));
        byte_num += WriteMessageAndShift(p, static_cast<float>(fuse_points[index].altitude_relative_min_));
        byte_num += WriteMessageAndShift(p, static_cast<float>(fuse_points[index].altitude_relative_max_));
        byte_num += WriteMessageAndShift(p, static_cast<double>(fuse_points[index].intensity_));
        byte_num += WriteMessageAndShift(p, static_cast<unsigned int>(fuse_points[index].count_));
        byte_num += WriteMessageAndShift(p, static_cast<unsigned char>(fuse_points[index].r_));
        byte_num += WriteMessageAndShift(p, static_cast<unsigned char>(fuse_points[index].g_));
        byte_num += WriteMessageAndShift(p, static_cast<unsigned char>(fuse_points[index].b_));
    }

    AINFO << boost::format("Saved %d fused points in buffer, occupying %u bytes") % fuse_points.size() % byte_num;
    return byte_num;
}

void GridMatrix::load_buffer(const std::vector<unsigned char>& buf) {
    const double matrix_resolution = option_->resolution * option_->matrix_size;
    double matrix_min_x = static_cast<double>(matrix_index_.index_x_ * matrix_resolution);
    double matrix_min_y = static_cast<double>(matrix_index_.index_y_ * matrix_resolution);

    int buf_len = buf.size();
    auto* p = const_cast<unsigned char*>(&buf[0]);

    // version 0: 2024-04-25 9.0.0-rc-r13前版本
    // version 1: 9.1.0前版本
    // version 2: 9.1.0之后版本
    auto get_bin_version = [&](const std::vector<unsigned char>& buf) -> int {
        const size_t ver0_size = sizeof(unsigned char) * 5 + sizeof(float) * 3 + sizeof(int) * 1;
        const size_t ver1_size = sizeof(unsigned char) * 4 + sizeof(float) * 3 + sizeof(int) * 1 + sizeof(double) * 1;
        const size_t ver2_size = GetBinPointSize();
        std::vector<size_t> size_list = {ver0_size, ver1_size, ver2_size};
        auto len = buf.size();
        for (int ver = 0; ver < 2; ++ver) {
            if (len % size_list[ver] != 0) {
                continue;
            }
            bool is_matched = true;
            for (size_t i = 0; i < len; i += size_list[ver]) {
                if (buf[i] != '\0') {
                    is_matched = false;
                    break;
                }
            }
            if (is_matched) {
                AWARN << "use bin version: " << ver;
                return ver;
            }
        }
        AWARN << "use default bin version: 2";
        return 2;
    };
    int bin_version = get_bin_version(buf);

    for (int i = 0; i < buf_len;) {
        /* layer */
        unsigned char layer_id = 0;
        i += LoadMessageAndShift<unsigned char>(p, layer_id);

        /* x y z */
        float x = 0, y = 0, z = 0, z_relative = 0, z_relative_var = 0, z_relative_min = 0, z_relative_max = 0;
        i += LoadMessageAndShift(p, x);
        i += LoadMessageAndShift(p, y);
        i += LoadMessageAndShift(p, z);
        if (bin_version == 2) {
            i += LoadMessageAndShift(p, z_relative);
            i += LoadMessageAndShift(p, z_relative_var);
            i += LoadMessageAndShift(p, z_relative_min);
            i += LoadMessageAndShift(p, z_relative_max);
        }

        /* intensity */
        double intensity = 0;
        if (bin_version > 0) {
            i += LoadMessageAndShift(p, intensity);
        } else {
            unsigned char intensity_c = 0;
            i += LoadMessageAndShift(p, intensity_c);
            intensity = static_cast<double>(intensity_c);
        }

        /* count */
        unsigned int count = 0;
        i += LoadMessageAndShift(p, count);

        /* rgb */
        unsigned char r = 0, g = 0, b = 0;
        i += LoadMessageAndShift(p, r);
        i += LoadMessageAndShift(p, g);
        i += LoadMessageAndShift(p, b);

        double x_d = (double)x + (double)matrix_min_x;
        double y_d = (double)y + (double)matrix_min_y;

        ADEBUG << boost::format("layer id=%d (%.2f, %.2f, %.2f) d=%d, count=%d, (r,g,b)=(%d, %d, %d)")
                        % (unsigned int)layer_id % x % y % z % (unsigned int)intensity % count % (unsigned int)r
                        % (unsigned int)g % (unsigned int)b;
        this->SetValue(
                x_d,
                y_d,
                z,
                z_relative,
                z_relative_var,
                z_relative_min,
                z_relative_max,
                count,
                intensity,
                r,
                g,
                b,
                layer_id);
    }
}

bool GridMatrix::SaveBinaryMatrixFile(const std::string& dir_path) {
    if (!cyber::common::EnsureDirectory(dir_path)) {
        AERROR << "create binary directory fail, path = " << dir_path;
        return false;
    }

    std::string binary_file_path = dir_path + "/" + GetMatrixKey() + ".bin";
    std::vector<unsigned char> matrix_value_buffer;
    if (!save_buffer(matrix_value_buffer)) {
        AERROR << "read binary info fail from matrix " << GetMatrixKey();
        return false;
    }

    std::ofstream output_file_stream(binary_file_path);
    std::copy(
            matrix_value_buffer.begin(),
            matrix_value_buffer.end(),
            std::ostream_iterator<unsigned char>(output_file_stream));
    output_file_stream.close();
    AINFO << "binary file saved, filename = " << binary_file_path << ", size = " << matrix_value_buffer.size()
          << " byte";
    return true;
}

bool GridMatrix::LoadBinaryMatrixFile(const std::string& dir_path) {
    std::vector<unsigned char> buf;
    if (!boost::filesystem::exists(dir_path) || !boost::filesystem::is_directory(dir_path)) {
        AERROR << "load matrix fail: directory not exist, " << dir_path;
        return false;
    }

    std::string binary_file_path = dir_path + "/" + GetMatrixKey() + ".bin";
    std::ifstream input_file_stream(binary_file_path, std::ios::binary);
    input_file_stream.seekg(0, std::ios::end);
    buf.resize(input_file_stream.tellg());
    input_file_stream.seekg(0);
    input_file_stream.read(reinterpret_cast<char*>(buf.data()), buf.size());
    input_file_stream.close();

    if (buf.size() == 0) {
        AERROR << "load matrix fail: load buffer not success, " << dir_path;
        return false;
    }

    (void)load_buffer(buf);

    return true;
}

}  // namespace tile_map_images_creator
}  // namespace apollo
