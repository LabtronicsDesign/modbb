// data_struct.h
#ifndef DATA_STRUCT_H
#define DATA_STRUCT_H

// Define your struct
typedef struct {
	uint16_t address;
    uint16_t buffer[8];
    uint16_t size;
} dispenseAddressValuePair;

#endif  // DATA_STRUCT_H
