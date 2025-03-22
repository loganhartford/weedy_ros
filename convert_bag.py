import os
import csv
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

def flatten_msg(msg, prefix=''):
    flat_dict = {}
    for field_name in msg.get_fields_and_field_types():
        value = getattr(msg, field_name)
        full_name = f"{prefix}.{field_name}" if prefix else field_name

        # Nested message
        if hasattr(value, 'get_fields_and_field_types'):
            flat_dict.update(flatten_msg(value, full_name))
        # Basic array
        elif isinstance(value, (list, tuple)) and all(isinstance(v, (int, float, str, bool)) for v in value):
            for i, v in enumerate(value):
                flat_dict[f"{full_name}[{i}]"] = v
        # Non-nested field
        else:
            flat_dict[full_name] = value
    return flat_dict

def sanitize(value):
    if isinstance(value, str):
        return value.replace('\n', ' ').replace('\r', ' ')
    return str(value).replace('\n', ' ').replace('\r', ' ')

def bag_to_csv(bag_path, output_dir):
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    topics_and_types = reader.get_all_topics_and_types()
    type_map = {t.name: get_message(t.type) for t in topics_and_types}

    os.makedirs(output_dir, exist_ok=True)
    writers = {}

    while reader.has_next():
        topic, data, t = reader.read_next()
        msg_type = type_map[topic]
        msg = deserialize_message(data, msg_type)

        flat_msg = flatten_msg(msg)

        if topic not in writers:
            file = open(os.path.join(output_dir, f"{topic.strip('/').replace('/', '_')}.csv"), 'w', newline='')
            writer = csv.writer(file)
            writers[topic] = (writer, file)
            writer.writerow(['timestamp'] + list(flat_msg.keys()))

        writer, _ = writers[topic]
        writer.writerow([t] + [sanitize(v) for v in flat_msg.values()])

    for writer, file in writers.values():
        file.close()

if __name__ == '__main__':
    import sys
    bag_path = sys.argv[1] if len(sys.argv) > 1 else 'my_bag'
    output_dir = f'{bag_path}_csv'
    rclpy.init()
    bag_to_csv(bag_path, output_dir)
    rclpy.shutdown()
