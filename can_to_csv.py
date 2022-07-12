data_name = ""
component_name = ""
can_ID = 0
bit_or_byte = 0
start_byte = 0
stop_byte = 0
start_bit = 0
stop_bit = 0
endian = 0
scale = 0
offset = 0
output = ""
ifsigned = 0

data_name = input("Enter data name: ");
component_name = input("Enter component name: ")
can_ID = input("Enter can_ID: ")
bit_or_byte = int(input("Enter bit or byte (bit enter 1, byte enter 2): "))
if bit_or_byte == int(1):
    start_byte = int(input("Enter start byte: "))
    start_bit = int(input("Enter start bit: "))
    stop_bit = int(input("Enter how many bits it contains: "))
    stop_bit += int(start_bit)
elif bit_or_byte == int(2):
    start_byte = int(input("Enter start byte: "))
    stop_byte = int(input("Enter how many bytes it contains: "));
    stop_byte += int(start_byte)
endian = int(input("Enter big endian or small endian (big enter 1, small enter 2): "))
endian += 2
ifsigned = input("Enter if signed (unsigned enter 0, signed enter 1) : ")
scale = input("Enter scale: ")
offset = input("Enter offset: ")

print("finished")
#"""
output = str(data_name) + "," \
        + str(component_name) + "," \
        + str(can_ID) + "," \
        + str(bit_or_byte) + "," \
        + str(endian)  + "," \
        + str(start_byte)  + "," \
        + str(stop_byte)  + "," \
        + str(start_bit)  + "," \
        + str(stop_bit) + "," \
        + str(ifsigned) + "," \
        + str(scale) + "," \
        + str(offset)
print("output:\n" + output)
print("end")
