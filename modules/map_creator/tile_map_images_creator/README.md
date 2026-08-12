# 地图制作二维底图生成

## 命令行运行

### 命令行样例

基本格式：

```bash
tile_map_images_creator [-c configure_file_path | -i input_records_directory | -o output_images_directory].
```

示例：

- 不指定任何项。则使用默认的配置文件路径`/apollo/modules/map_creator/tile_map_images_creator/conf/image_creator_conf.pb.txt`，输入输出路径使用配置文件内指定的

```bash
tile_map_images_creator
```

- 指定的配置文件路径，不额外指定输入输出路径。此时输入输出路径都使用指定配置文件内input_dir和images_output_dir项

```bash
tile_map_images_creator -c test_conf.pb.txt
```

- 指定的输入输出文件夹路径，此时使用默认的配置文件，指定的输入输出文件夹将覆盖配置文件内的input_dir和images_output_dir项

```bash
tile_map_images_creator -i 输入record文件夹目录 -o 输出图像目录
```

- 指定配置文件路径、输入输出路径。此时使用指定的配置文件，但里面的input_dir和images_output_dir将会被指定的输入输出路径覆盖

```bash
tile_map_images_creator -c test_conf.pb.txt -i 输入record文件夹目录 -o 输出图像目录
```

### 命令行参数项

-c 指定配置文件路径。不填则为`/apollo_workspace/modules/map_creator/tile_map_images_creator/conf/image_creator_conf.pb.txt`

-i 指定输入的record文件夹目录。会覆盖配置文件中`input_dir`项，如不填则采用配置文件指定的路径。

-o 指定图像输出目录。同样会覆盖配置文件中`images_output_dir`项，如不填则采用配置文件指定的路径

## 配置文件

### 配置项解释

配置文件指以`modules/map_creator/tile_map_images_creator/conf/image_creator_conf.pb.txt`为示例的pb.txt格式的配置项，可以通过-c指定特定的配置文件

- reader_conf。描述使用到的通道名称，包含世界坐标系下的点云通道，定位位姿信息输入通道，通常不需要修改。
- point_cloud_processing_conf。点云处理配置项
    - intensity_converter_conf。sigmoid变化器的配置项。包含k和x0两项，一般只需要关注x0项，**将x0调大一般会让图像变暗，反之则变亮**；根据目前一些的测试结果，通常在如树荫底下的较阴暗的环境下，x0调整为11.0~13.0比较好；通常在开放、光线比较充足的场景下，x0调整为18.0~20.0比较好。
    - filter_conf。点云过滤的配置项，包含高度过滤项和距离过滤项。高度过滤的目的是过滤掉非地面点，其阈值是和pose的z轴高度的相对值。距离过滤项的目的是过滤掉扫到车身的点和较远的点，阈值是点云距离pose中心的距离的有效范围。
- matrix_generator_conf。包含精度和初始生成的瓦片最大层级，精度指最大层级的瓦片每个图像的每个像素包含的物理距离，通常不需要修改。地图通常分为0~4层共五层，每张瓦片分辨率都是1024\*1024。第4层瓦片每个像素点的物理距离为精度的大小，也就是0.03125\*0.3125m^2，第4层瓦片每张大小表示32\*32m^2的物理范围。接着上层瓦片的表示物理范围的长宽都分别为下层瓦片的两倍，如第三层级表示64\*64的物理范围，第二层级表示128\*128的范围，依次类推。
- input_output_conf，包含了输入输出目录和LRU缓存配置
    - input_dir指输入record路径，会被-i覆盖
    - bin_output_dir指缓存用到的二进制文件的文件夹路径，空间占用可能会比较大，在开始执行前会被清空
    - images_output_dir指最终输出图像的文件夹路径，会被-o覆盖
    - LRU项，可配置内存中最多可存储多少个瓦片

- sample_distance，目的是过滤距离太近的点云帧，限定用到的两帧点云之间的最小距离间隔，如果所有点云都需要用于渲染，则设为0.0
- debug_conf，目前只有一项，会输出过滤后点云的pcd文件
