/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include "gtest/gtest.h"

#include "cstdlib"

#include "modules/map_creator/tile_map_images_creator/grid_map/grid_matrix.h"

namespace apollo {
namespace tile_map_images_creator {

class CheckMatrixSaveAndLoadTest : public testing::Test {
public:
    virtual void SetUp() override {
        p_matrix = std::make_shared<GridMatrix>();
        p_matrix_check = std::make_shared<GridMatrix>();
    }
    std::shared_ptr<GridMatrix> p_matrix, p_matrix_check;
};

TEST_F(CheckMatrixSaveAndLoadTest, save_and_load_test) {
    std::shared_ptr<GridOption> option = std::make_shared<GridOption>();
    option->matrix_size = 1024;
    option->resolution = 0.3125;
    option->resolution_id = 0;
    p_matrix->Init(option);
    std::cout << "origin matrix inited" << std::endl;

    srand(time(nullptr));
    for (int i = 0; i < 1000; ++i) {
        int x = rand() % 1024;
        int y = rand() % 1024;
        double z = rand() % 10000 * 0.01;
        int id = rand() % 256;

        auto &point = p_matrix->get_mutable_map_cell(x, y);
        point.SetValue(z, 1, id, id, id, id);
    }
    std::string bin_file_path = "/apollo_workspace/test.bin";
    EXPECT_TRUE(p_matrix->SaveBinaryMatrixFile(bin_file_path));

    p_matrix_check->Init(option);
    p_matrix_check->LoadBinaryMatrixFile(bin_file_path);

    std::vector<GridCellSingle> vec_origin, vec_check;
    p_matrix->LoadFusePoints(vec_origin);
    p_matrix_check->LoadFusePoints(vec_check);

    EXPECT_EQ(vec_check.size(), vec_origin.size());
    std::cout << "check point size = " << vec_check.size() << std::endl;
    for (int i = 0; i < std::min(vec_check.size(), vec_origin.size()); ++i) {
        GridCellSingle &p_origin = vec_origin[i];
        GridCellSingle &p_check = vec_check[i];

        EXPECT_EQ(p_origin.count_, p_check.count_);
        EXPECT_FLOAT_EQ(p_origin.intensity_, p_check.intensity_);
        EXPECT_FLOAT_EQ(p_origin.altitude_, p_check.altitude_);
        EXPECT_EQ(p_origin.index_x_, p_check.index_x_);
        EXPECT_EQ(p_origin.index_y_, p_check.index_y_);
    }
}

}  // namespace tile_map_images_creator
}  // namespace apollo