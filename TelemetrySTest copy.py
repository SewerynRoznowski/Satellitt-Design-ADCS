import struct

# Float value
float_value = 3.14159

# Convert float to bytes
bytes_array = struct.pack('f', float_value)

# Print the bytes array
print(bytes_array)