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

#pragma once

#include <cassert>
#include <memory>
#include <vector>

#include "modules/map_creator/tile_map_images_creator/grid_map/grid_cell.h"
#include "modules/map_creator/tile_map_images_creator/grid_map/grid_cell_single.h"
#include "modules/map_creator/tile_map_images_creator/grid_map/grid_option.h"

namespace apollo {
namespace tile_map_images_creator {

struct MatrixIndex {
    int index_x_;
    int index_y_;
    MatrixIndex() = default;
    MatrixIndex(int index1, int index2) {
        index_x_ = index1;
        index_y_ = index2;
    }
    MatrixIndex(const MatrixIndex& matrix_index) {
        index_x_ = matrix_index.index_x_;
        index_y_ = matrix_index.index_y_;
    }
    ~MatrixIndex() = default;
    bool operator==(const MatrixIndex& matrix_index) const {
        return (index_x_ == matrix_index.index_x_ && index_y_ == matrix_index.index_y_);
    }
    MatrixIndex& operator=(const MatrixIndex& matrix_index) {
        index_x_ = matrix_index.index_x_;
        index_y_ = matrix_index.index_y_;
        return *this;
    }
    bool operator<(const MatrixIndex& other) const {
        if (index_x_ == other.index_x_) {
            return index_y_ < other.index_y_;
        }
        return index_x_ < other.index_x_;
    }
};

class GridMatrix {
public:
    GridMatrix() = default;
    GridMatrix(const GridMatrix& matrix);
    GridMatrix(std::shared_ptr<GridOption> option);
    void Init(std::shared_ptr<GridOption> option);

    /**@brief Add a sample into a layer of matrix. */
    void SetValue(
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
            int layer = 0);

    inline unsigned int get_rows() const {
        return rows_;
    }
    inline unsigned int get_cols() const {
        return cols_;
    }
    inline int get_resolution_id() const {
        return resolution_id_;
    }
    inline MatrixIndex get_index() const {
        return matrix_index_;
    }
    inline void set_index(MatrixIndex& matrix_index) {
        matrix_index_ = matrix_index;
    }

    GridCell& get_mutable_map_cell(int row, int col) {
        assert(row < static_cast<int>(rows_) && row >= 0);
        assert(col < static_cast<int>(cols_) && col >= 0);
        return map_cells_[row * cols_ + col];
    }

    const GridCell& get_const_map_cell(int row, int col) const {
        assert(row < static_cast<int>(rows_) && row >= 0);
        assert(col < static_cast<int>(cols_) && col >= 0);
        return map_cells_[row * cols_ + col];
    }

    void LoadFusePoints(std::vector<GridCellSingle>& fuse_points);

    /**@brief create binary file for matrix*/
    bool SaveBinaryMatrixFile(const std::string& dir_path);
    /**@brief Load binary file for matrix*/
    bool LoadBinaryMatrixFile(const std::string& dir_path);
    /**@brief get index of matrix in string format*/
    std::string GetMatrixKey() const;

    /**@brief Reset all cells of matrix with default value*/
    void Reset();

private:
    size_t GetBinPointSize() const;
    /**@brief Create binary buf. */
    unsigned int save_buffer(std::vector<unsigned char>& buf);
    /**@brief Load buffer from binary file*/
    void load_buffer(const std::vector<unsigned char>& buf);

    /**@brief The zone id of map. */
    int zone_id_;
    /**@brief The resolution of map. */
    float resolution_;
    /**@brief The resolution id of map. */
    int resolution_id_;
    /**@brief The number of rows. */
    unsigned int rows_;
    /**@brief The number of columns. */
    unsigned int cols_;
    /**@brief The matrix data structure. */
    std::vector<GridCell> map_cells_;
    /**@brief The index of matrix. */
    MatrixIndex matrix_index_;
    /**@brief The point in matrix. */
    std::vector<GridCellSingle> fuse_points_;
    /**@brief The option of map. */
    std::shared_ptr<GridOption> option_;
};

}  // namespace tile_map_images_creator
}  // namespace apollo