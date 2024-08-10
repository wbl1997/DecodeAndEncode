import os
import json
import struct

def ascii_to_binary_bin(ascii_file_path):
    with open(ascii_file_path, 'r') as ascii_file:
        lines = ascii_file.readlines()

    data_lines = []
    is_data_section = False

    for line in lines:
        if line.startswith('DATA'):
            is_data_section = True
        elif is_data_section:
            data_lines.append(line.strip())

    binary_data = bytearray()

    for data_line in data_lines:
        x, y, z, intensity = data_line.split()
        binary_data.extend(struct.pack('ffff', float(x), float(y), float(z), float(intensity)))

    return binary_data

def load_simulation_data(json_path, jpg_path):
    with open(json_path, 'r') as json_file:
        data = json.load(json_file)

    with open(jpg_path, 'rb') as jpg_file:
        jpg_data = jpg_file.read()

    return data, jpg_data

def encode_to_binary(json_path, jpg_path, pcd_3d_path, pcd_2d_path, bin_path):
    simulation_data, jpg_data = load_simulation_data(json_path, jpg_path)
    pcd_3d = ascii_to_binary_bin(pcd_3d_path)
    pcd_2d = ascii_to_binary_bin(pcd_2d_path)

    with open(bin_path, 'wb') as bin_file:
        # Encode JSON data
        json_data = json.dumps(simulation_data).encode('utf-8')
        bin_file.write(struct.pack('I', len(json_data)))
        bin_file.write(json_data)

        # Encode 3D point cloud data
        bin_file.write(pcd_3d)

        # Encode 2D point cloud data
        bin_file.write(pcd_2d)

        # Encode JPEG image data
        bin_file.write(struct.pack('I', len(jpg_data)))
        bin_file.write(jpg_data)

def main():
    json_path = input("Enter the path to the JSON file: ")
    jpg_path = input("Enter the path to the JPG file: ")
    pcd_3d_path = input("Enter the path to the 3D PCD file: ")
    pcd_2d_path = input("Enter the path to the 2D PCD file: ")
    bin_path = input("Enter the output path for the BIN file: ")

    # Ensure the output directory exists
    os.makedirs(os.path.dirname(bin_path), exist_ok=True)

    encode_to_binary(json_path, jpg_path, pcd_3d_path, pcd_2d_path, bin_path)

if __name__ == "__main__":
    main()
