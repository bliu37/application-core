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

#include "modules/map_creator/tile_map_images_creator/time_align/time_align_cache.hpp"

#include <iostream>
#include <memory>

#include "gtest/gtest.h"

namespace apollo {
namespace tile_map_images_creator {

class TimeAlignCacheTest : public testing::Test {};

TEST_F(TimeAlignCacheTest, time_align_cache_test_get_answer) {
    std::cout << "######### time_align_cache_test_get_answer test start #########" << std::endl;
    std::shared_ptr<TimeAlignCache<int>> p_time_align_test(
            new TimeAlignCache<int>([](std::shared_ptr<int> x) -> uint64_t { return *x; }));

    EXPECT_TRUE(p_time_align_test->Init());

    p_time_align_test->Insert(std::make_shared<int>(1));
    p_time_align_test->Insert(std::make_shared<int>(3));
    p_time_align_test->Insert(std::make_shared<int>(4));
    p_time_align_test->Insert(std::make_shared<int>(5));
    p_time_align_test->Insert(std::make_shared<int>(6));
    p_time_align_test->Insert(std::make_shared<int>(7));
    p_time_align_test->Insert(std::make_shared<int>(9));

    std::shared_ptr<int> res = std::make_shared<int>();
    p_time_align_test->FindNearest(2, res);
    EXPECT_EQ(*res, 1);

    p_time_align_test->FindNearest(5, res);
    EXPECT_EQ(*res, 5);
}

TEST_F(TimeAlignCacheTest, time_align_cache_test_get_nearest) {
    std::cout << "######### time_align_cache_test_get_nearest test start #########" << std::endl;
    std::shared_ptr<TimeAlignCache<int>> p_time_align_test(
            new TimeAlignCache<int>([](std::shared_ptr<int> x) -> uint64_t { return *x; }));

    EXPECT_TRUE(p_time_align_test->Init());

    p_time_align_test->Insert(std::make_shared<int>(1));
    p_time_align_test->Insert(std::make_shared<int>(2));
    p_time_align_test->Insert(std::make_shared<int>(3));
    p_time_align_test->Insert(std::make_shared<int>(4));
    p_time_align_test->Insert(std::make_shared<int>(5));
    p_time_align_test->Insert(std::make_shared<int>(9));
    p_time_align_test->Insert(std::make_shared<int>(10));

    std::shared_ptr<int> res = std::make_shared<int>();
    p_time_align_test->FindNearest(8, res);

    EXPECT_EQ(*res, 9);
}

TEST_F(TimeAlignCacheTest, time_align_cache_test_incoming) {
    std::cout << "######### time_align_cache_test_incoming test start #########" << std::endl;
    std::shared_ptr<TimeAlignCache<int>> p_time_align_test(
            new TimeAlignCache<int>([](std::shared_ptr<int> x) -> uint64_t { return *x; }));

    EXPECT_TRUE(p_time_align_test->Init());

    p_time_align_test->Insert(std::make_shared<int>(1));
    p_time_align_test->Insert(std::make_shared<int>(2));
    p_time_align_test->Insert(std::make_shared<int>(3));
    p_time_align_test->Insert(std::make_shared<int>(4));
    p_time_align_test->Insert(std::make_shared<int>(5));
    p_time_align_test->Insert(std::make_shared<int>(9));
    p_time_align_test->Insert(std::make_shared<int>(10));

    std::shared_ptr<int> res = std::make_shared<int>();
    EXPECT_FALSE(p_time_align_test->FindNearest(11, res));

    EXPECT_TRUE(p_time_align_test->HeadCheck());
    EXPECT_FALSE(p_time_align_test->HeadAndCacheCheck());

    p_time_align_test->Insert(std::make_shared<int>(12));
    EXPECT_TRUE(p_time_align_test->FindNearest(11, res));
}

}  // namespace tile_map_images_creator
}  // namespace apollo
