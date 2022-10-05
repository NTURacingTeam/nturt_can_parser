# YAML rule
###### tags: `software` `electrical_system` `NTURT`

Please put your can configuration `can.yaml` in the same directory as this documentation.

## Example

A simple can configuration with a frame and a byte and bit data.

```yaml=
can:
  frame:
    name: mcu_command
    id: 0x0C0
    is_extended_id: false
    dlc: 8
    period: 0
    dataset:
      data: # byte data
        name: torque_command
        is_signed: true
        is_byte: true
        is_little_endian: true
        start_byte: 0
        end_byte: 2
        default: 0
        resolution: 0.1
        offset: 0
      data: # bit data
        name: inverter_enable
        is_signed: false
        is_byte: false
        byte: 5
        start_bit: 0
        end_bit: 1
        default: 0
        resolution: 1
        offset: 0
```

## Explanation

Listed in the order of the example above.

### can:

- Root of the can file.
- type: none
- required: true
- raise: std::runtime_error
  - If not exist.

### frame:

- Root of this frame, should be at least one or more.
- type: none
- required: true
- raise: std::runtime_error
  - If not exist.

### name:

- Name of this frame.
- type: string
- required: true
- raise: std::runtime_error
  - If not exist.

### id:

- Id of this frame, should be written in hexadecimal fasion.
- type: int
- required: true
- raise: std::runtime_error
  - If not exist.

### is_extended_id:

- Determine if the id of this frame is extended id, the id is extended if is_extended_id is set to true even if the id is shorter or equal to 12 bit.
- type: boolean
- required: false
  - if not provided: It will be true if the id is longer then 12 bit, or false if the it is equal or shorter to 12 bit.
- raise: std::runtime_error
  - If is_extended_id is set to false while the id is longer than 12 bit.

### dlc:

- The number of byte of the data stored in this frame, 
- type: int
- required: false
  - If not provided: It will be equal to the hightest byte occupied by the data stored in this frame.
- raise: std::runtime_error
  - If the hightest byte occupied by the data stored in this frame is greater to dlc.

### period:

- The period at which this frame is sent to can bus, if set to 0, the frame will not be sent to can bus.
- type: double
- required: false
  - Default to 0.

### dataset:

- The root of dataset.
- type: none
- required: true
- raise: std::runtime_error
  - If not exist.

### data:

- The root of this data, should be at least one or more.
- type: none
- required: true
- raise: std::runtime_error
  - If not exist.

### name:

- The name of this data.
- type: string
- required: true
- raise: std::runtime_error
  - If not exist.

### is_signed:

- Determine if this data is signed.
- type: boolean
- required: true
- raise: std::runtime_error
  - If not exist.

### is_byte:

- Determine if this data is byte or bit data.
- type: boolean
- required: true
- raise: std::runtime_error
  - If not exist.

### is_little_endian:

- Determine if this data is little or big endian, for little endian, the value of the data is calculated as (first byte) + 256 * (second byte) + 256 ^ 2 *(third byte) ... It is calculated in the reversed order if it is big endian.
- type: boolean
- required: only required
  - This flag is only required if both is_byte is set to true and the data is two byte or longer.
- raise: std::runtime_error
  - If this flag exist while either is_byte is set to false or the data is shorter than on byte.
  - If this flag does not exist if both is_byte is set to true and the data is two bit or longer.

### start_byte:

- The number of byte that start containing this data, counted inclusively, or [start_byte, end_byte).
- type: int
- required: only required
- raise: std::runtime_error
  - If exist while is_byte is set to false.
  - If not exist while is_byte is set to true.
- rasie std::range_error
  - If it is less than 0 or greater to 7.

### end_byte:

- The number of byte that end containing this data, counted exclusively, or [start_byte, end_byte).
- type: int
- required: only required
  - Only required if is_byte is set to true.
- raise: std::runtime_error
  - If exist while is_byte is set to false.
  - If not exist while is_byte is set to true.
- rasie std::range_error
  - If it is less or equal to end_byte or greater to 8.

### byte:

- The number of byte that containing this data.
- type: int
- required: only required
  - Only required if is_byte is set to false.
- raise: std::runtime_error
  - If exist while is_byte is set to true.
  - If not exist while is_byte is set to false.
- rasie std::range_error
  - If it is less than 0 or greater to 7.

### start_bit:

- The number of bit that start containing this data, counted inclusively, or [start_bit, end_bit).
- type: int
- required: only required
  - Only required if is_byte is set to false.
- raise: std::runtime_error
  - If exist while is_byte is set to true.
  - If not exist while is_byte is set to false.
- rasie std::range_error
  - If it is less than 0 or greater to 7.

### end_bit:

- The number of bit that end containing this data, counted exclusively, or [start_bit, end_bit).
- type: int
- required: only required
  - Only required if is_byte is set to false.
- raise: std::runtime_error
  - If exist while is_byte is set to true.
  - If not exist while is_byte is set to false.
- rasie std::range_error
  - If it is less or equal to end_bit or greater to 8.

### default:

- The dafault value of this data.
- type: double
- required: false
  - Default to 0.

### resolution:

- The resolution of this data.
- type: double
- required: false
  - Default to 1.

### offset:

- The offset of this data.
- type: double
- required: false
  - Default to 0.

### Special case

- Raise std::runtime_error if two or more data have overlaping bit.

## Note

Random tags that does not belong to the list above will not be checked or parsed.
