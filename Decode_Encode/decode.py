import os
import json
import struct
import sys

def binary_bin_to_ascii_pcd(bin_file, ascii_file_path, header, num_points):
    point_size = struct.calcsize('ffff')
    with open(ascii_file_path, 'w') as ascii_file:
        ascii_file.write(header)
        for i in range(num_points):
            point_data = bin_file.read(point_size)
            x, y, z, intensity = struct.unpack('ffff', point_data)
            ascii_file.write('{:.6f} {:.6f} {:.6f} {:.6f}\n'.format(x, y, z, intensity))

def decode_bin_file(bin_path):
    decode_dir = os.path.dirname(bin_path)
    frame_index = os.path.splitext(os.path.basename(bin_path))[0].split('_')[-1]

    json_path = os.path.join(decode_dir, 'JSON', f'frame_{frame_index}.json')
    jpg_path = os.path.join(decode_dir, 'JPG', f'frame_{frame_index}.jpg')
    pcd_3d_path = os.path.join(decode_dir, '3D', f'frame_{frame_index}.pcd')
    pcd_2d_path = os.path.join(decode_dir, '2D', f'frame_{frame_index}.pcd')

    os.makedirs(os.path.join(decode_dir, 'JSON'), exist_ok=True)
    os.makedirs(os.path.join(decode_dir, 'JPG'), exist_ok=True)
    os.makedirs(os.path.join(decode_dir, '3D'), exist_ok=True)
    os.makedirs(os.path.join(decode_dir, '2D'), exist_ok=True)

    with open(bin_path, 'rb') as bin_file:
        # Decode JSON data
        json_length = struct.unpack('I', bin_file.read(4))[0]
        json_data = json.loads(bin_file.read(json_length))
        with open(json_path, 'w') as json_file:
            json.dump(json_data, json_file, indent=4)

        # Decode 3D point cloud data
        pcd_3d_header = '''# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH {width}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {points}
DATA ascii
'''.format(width=json_data['3dpcd_num'], points=json_data['3dpcd_num'])
        binary_bin_to_ascii_pcd(bin_file, pcd_3d_path, pcd_3d_header, json_data['3dpcd_num'])

        # Decode 2D point cloud data
        pcd_2d_header = '''# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH {width}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {points}
DATA ascii
'''.format(width=json_data['2dpcd_num'], points=json_data['2dpcd_num'])
        binary_bin_to_ascii_pcd(bin_file, pcd_2d_path, pcd_2d_header, json_data['2dpcd_num'])

        # Decode JPEG image data
        jpg_length = struct.unpack('I', bin_file.read(4))[0]
        jpg_data = bin_file.read(jpg_length)
        with open(jpg_path, 'wb') as jpg_file:
            jpg_file.write(jpg_data)


def main():
    if len(sys.argv) != 2:
        print("Usage: python script.py <bin_file_path>")
        return

    bin_file_path = sys.argv[1]
    if not os.path.isfile(bin_file_path):
        print(f"File {bin_file_path} does not exist.")
        return

    decode_bin_file(bin_file_path)


if __name__ == "__main__":
    main()
