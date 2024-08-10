# Decode_Encode 项目

本项目包含了一些用于编码和解码、生成热成像图片和点云数据（PCD）的Python脚本。

## 目录结构

- `Decode_Encode/decode.py`
- `Decode_Encode/decode_multi.py`
- `Decode_Encode/encode.py`
- `Decode_Encode/encode_multi.py`
- `Decode_Encode/iray.py`
- `Decode_Encode/PCD.py`

## 脚本功能说明

### 1. `decode.py`

- **功能**: 用于单帧解码。
- **用途**: 读取一个编码的二进制文件，并解码为JSON、2D PCD、3D PCD和JPEG图像格式的文件。

### 2. `decode_multi.py`

- **功能**: 用于多帧解码。
- **用途**: 批量读取多个编码的二进制文件，并解码为JSON、2D PCD、3D PCD和JPEG图像格式的文件。

### 3. `encode.py`

- **功能**: 用于单帧编码。
- **用途**: 将单个帧的JSON数据、2D PCD、3D PCD和JPEG图像编码为一个二进制文件。

### 4. `encode_multi.py`

- **功能**: 用于多帧编码。
- **用途**: 批量处理多个帧的JSON数据、2D PCD、3D PCD和JPEG图像，并编码为多个二进制文件。

### 5. `iray.py`

- **功能**: 用于生成JPG格式的热成像图片。
- **用途**: 处理红外数据，并生成相应的JPG格式的热成像图片。

### 6. `PCD.py`

- **功能**: 用于生成2D的PCD、3D的PCD和对应的JSON文件。
- **用途**: 处理点云数据，生成2D和3D格式的PCD文件，以及包含元数据信息的JSON文件。

## 使用方法

- 每个脚本可以独立运行，根据脚本功能实现相应的操作。
- 在运行 `encode.py` 和 `decode.py` 时，需要指定输入文件和输出文件的路径。
- 对于批量操作，请使用 `encode_multi.py` 和 `decode_multi.py`。

## 注意事项

- 请确保输入文件的路径和格式正确，以避免脚本运行错误。
- 对于生成的PCD文件，请使用兼容的点云处理工具进行可视化。

---

**作者**: levy

**版本**: 1.0.0
