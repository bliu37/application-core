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

#include "map_validator.h"

#include "gtest/gtest.h"

#include "string"

namespace apollo {
namespace map_tool {

class MapValidatorTest : public testing::Test {
public:
    virtual void SetUp() override {
        FLAGS_alsologtostderr = 1;

        checker_config_path_
                = "/apollo_workspace/modules/map_creator/map_tool/conf/map_validator_checker_conf.pb.txt";
        errcode_config_path_
                = "/apollo_workspace/modules/map_creator/map_tool/conf/map_validator_errcode_conf.pb.txt";

        map_bin_path_ = "/apollo/modules/map/data/apollo_virutal_map/base_map.bin";
        p_map_validator_ = std::make_shared<MapValidator>();
    }
    std::shared_ptr<MapValidator> p_map_validator_;
    std::string checker_config_path_, errcode_config_path_, map_bin_path_;
};

TEST_F(MapValidatorTest, running_test) {
    bool ret = p_map_validator_->Init(checker_config_path_, errcode_config_path_, map_bin_path_);
    EXPECT_TRUE(ret);

    ValidatorResult result;
    EXPECT_TRUE(p_map_validator_->Check(result));

    if (result.error_message_size() > 0) {
        AWARN << "error message receive : " << result.DebugString();
    }
}

TEST_F(MapValidatorTest, signal_overlap_error_test) {
    apollo::hdmap::Map map_instance;
    EXPECT_TRUE(apollo::cyber::common::GetProtoFromBinaryFile(map_bin_path_, &map_instance));

    auto new_lane = map_instance.add_lane();
    new_lane->mutable_id()->set_id("test_lane_id_1");
    auto new_curve = new_lane->mutable_central_curve();
    auto new_segment = new_curve->add_segment()->mutable_line_segment();
    auto p0 = new_segment->add_point();
    p0->set_x(100), p0->set_y(100);
    auto p1 = new_segment->add_point();
    p1->set_x(200), p1->set_y(200);

    auto new_polygon = new hdmap::Polygon();
    new_polygon->add_point()->CopyFrom(*p0);
    new_polygon->add_point()->CopyFrom(*p1);
    auto polygon_p3 = new_polygon->add_point();
    polygon_p3->set_x(100);
    polygon_p3->set_y(300);

    // test 1-1 红绿灯 没有overlap id
    auto new_signal = map_instance.add_signal();
    new_signal->mutable_id()->set_id("test_signal_id_1-1");  // test 1 没有overlap id
    new_signal->add_stop_line()->CopyFrom(*new_curve);

    // test 1-2 减速带 没有overlap id
    auto new_speed_bump = map_instance.add_speed_bump();
    new_speed_bump->mutable_id()->set_id("test_speed_bump_id_1-2");  // test 1 没有overlap id
    new_speed_bump->add_position()->CopyFrom(*new_curve);

    // test 1-3 停车位 没有overlap id
    auto new_parking = map_instance.add_parking_space();
    new_parking->mutable_id()->set_id("test_parking_space_id_1-3");
    p0 = new_parking->mutable_polygon()->add_point();
    p0->set_x(0), p0->set_y(0);
    p1 = new_parking->mutable_polygon()->add_point();
    p1->set_x(0), p1->set_y(8);
    auto p2 = new_parking->mutable_polygon()->add_point();
    p2->set_x(3), p2->set_y(8);
    auto p3 = new_parking->mutable_polygon()->add_point();
    p3->set_x(3), p3->set_y(0);

    // test 1-4 人行横道
    auto new_crosswalk = map_instance.add_crosswalk();
    new_crosswalk->mutable_id()->set_id("test_crosswalk_id_1-4");
    new_crosswalk->mutable_polygon()->CopyFrom(*new_polygon);

    // test 2 overlap没有object
    new_signal = map_instance.add_signal();
    new_signal->mutable_id()->set_id("test_signal_id_2");
    new_signal->add_stop_line()->CopyFrom(*new_curve);
    auto new_overlap_id = new_signal->add_overlap_id();
    new_overlap_id->set_id("test_overlap_1");
    auto new_overlap = map_instance.add_overlap();
    new_overlap->mutable_id()->set_id("test_overlap_1");

    // test 3 overlap没关联道路不是道路信息
    new_signal = map_instance.add_signal();
    new_signal->mutable_id()->set_id("test_signal_id_3");
    new_signal->add_stop_line()->CopyFrom(*new_curve);
    new_overlap_id = new_signal->add_overlap_id();
    new_overlap_id->set_id("test_overlap_2");
    new_overlap = map_instance.add_overlap();
    new_overlap->mutable_id()->set_id("test_overlap_2");
    auto new_object = new_overlap->add_object();
    new_object->mutable_id()->set_id("test_lane_id_1");
    new_object->mutable_crosswalk_overlap_info()->New();

    // test 4 正例
    new_signal = map_instance.add_signal();
    new_signal->mutable_id()->set_id("test_signal_id_4");
    new_signal->add_stop_line()->CopyFrom(*new_curve);
    new_overlap_id = new_signal->add_overlap_id();
    new_overlap_id->set_id("test_overlap_3");
    new_overlap = map_instance.add_overlap();
    new_overlap->mutable_id()->set_id("test_overlap_3");
    new_object = new_overlap->add_object();
    new_object->mutable_id()->set_id("test_lane_id_1");
    new_object->mutable_lane_overlap_info()->New();

    bool ret = p_map_validator_->Init(checker_config_path_, errcode_config_path_, map_instance);
    EXPECT_TRUE(ret);

    ValidatorResult result;
    EXPECT_TRUE(p_map_validator_->Check(result));

    std::set<std::string> err_desc;
    for (const auto& err : result.error_message()) {
        err_desc.insert(err.element_id());
        if (err.element_id() == "test_signal_id_1-1") {
            EXPECT_EQ(err.item_code(), SignalCannotAssociateLaneError);
        } else if (err.element_id() == "test_speed_bump_id_1-2") {
            EXPECT_EQ(err.item_code(), SpeedBumpCannotAssociateLaneError);
        } else if (err.element_id() == "test_parking_space_id_1-3") {
            EXPECT_EQ(err.item_code(), ParkingSpaceCannotAssociateLaneError);
        } else if (err.element_id() == "test_crosswalk_id_1-4") {
            EXPECT_EQ(err.item_code(), CrossWalkCannotAssociateLaneError);
        }
    }
    AWARN << "error message receive : " << result.DebugString();
    EXPECT_TRUE(err_desc.count("test_signal_id_1-1"));
    EXPECT_TRUE(err_desc.count("test_speed_bump_id_1-2"));
    EXPECT_TRUE(err_desc.count("test_parking_space_id_1-3"));
    EXPECT_TRUE(err_desc.count("test_crosswalk_id_1-4"));
    EXPECT_TRUE(err_desc.count("test_signal_id_2"));
    EXPECT_TRUE(err_desc.count("test_signal_id_3"));
    EXPECT_FALSE(err_desc.count("test_signal_id_4"));
}

TEST_F(MapValidatorTest, no_lane_single_element) {
    apollo::hdmap::Map map_instance;
    //    EXPECT_TRUE(apollo::cyber::common::GetProtoFromBinaryFile(map_bin_path_, &map_instance));

    AINFO << "no lane test begin";

    auto new_curve = new hdmap::Curve();
    auto new_segment = new_curve->add_segment()->mutable_line_segment();
    auto p0 = new_segment->add_point();
    p0->set_x(100), p0->set_y(100);
    auto p1 = new_segment->add_point();
    p1->set_x(200), p1->set_y(200);

    auto new_polygon = new hdmap::Polygon();
    new_polygon->add_point()->CopyFrom(*p0);
    new_polygon->add_point()->CopyFrom(*p1);
    auto polygon_p3 = new_polygon->add_point();
    polygon_p3->set_x(100);
    polygon_p3->set_y(300);

    // test 1-1 红绿灯 没有overlap id
    auto new_signal = map_instance.add_signal();
    new_signal->mutable_id()->set_id("test_signal_id_1-1");  // test 1 没有overlap id
    new_signal->add_stop_line()->CopyFrom(*new_curve);

    // test 1-2 减速带 没有overlap id
    auto new_speed_bump = map_instance.add_speed_bump();
    new_speed_bump->mutable_id()->set_id("test_speed_bump_id_1-2");  // test 1 没有overlap id
    new_speed_bump->add_position()->CopyFrom(*new_curve);

    bool ret = p_map_validator_->Init(checker_config_path_, errcode_config_path_, map_instance);
    EXPECT_TRUE(ret);

    ValidatorResult result;
    EXPECT_TRUE(p_map_validator_->Check(result));

    std::set<std::string> err_desc;
    for (const auto& err : result.error_message()) {
        err_desc.insert(err.element_id());
        if (err.element_id() == "test_signal_id_1-1") {
            EXPECT_EQ(err.item_code(), SignalCannotAssociateLaneError);
        } else if (err.element_id() == "test_speed_bump_id_1-2") {
            EXPECT_EQ(err.item_code(), SpeedBumpCannotAssociateLaneError);
        } else if (err.element_id() == "test_parking_space_id_1-3") {
            EXPECT_EQ(err.item_code(), ParkingSpaceCannotAssociateLaneError);
        } else if (err.element_id() == "test_crosswalk_id_1-4") {
            EXPECT_EQ(err.item_code(), CrossWalkCannotAssociateLaneError);
        }
    }
    AWARN << "error message receive : " << result.DebugString();
    EXPECT_TRUE(err_desc.count("test_signal_id_1-1"));
    EXPECT_TRUE(err_desc.count("test_speed_bump_id_1-2"));
}

TEST_F(MapValidatorTest, running_sample) {
    // 1. 从文件中读取高精地图，如果此时未生成好高精地图，跳过这一步直接调用Init方法
    apollo::hdmap::Map map_instance;
    EXPECT_TRUE(apollo::cyber::common::GetProtoFromBinaryFile(map_bin_path_, &map_instance));

    // 2. 用高精地图实例初始化质检对象。需要用到两个配置文件和高精地图实例
    EXPECT_TRUE(p_map_validator_->Init(checker_config_path_, errcode_config_path_, map_instance));

    // 3. 调用质检方法，错误ValidatorResult返回，内含所有出错的错误项和等级
    ValidatorResult result;
    EXPECT_TRUE(p_map_validator_->Check(result));

    // 4. 可读取所有的错误信息
    for (int i = 0; i < result.error_message_size(); ++i) {
        AWARN << "err occur: " << result.error_message(i).DebugString();
    }
}

}  // namespace map_tool
}  // namespace apollo
