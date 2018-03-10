# iBeacon Sample Mongoose OS library


## Overview

This library is intended to be used as a sample for iBeacon advertising.


## Usage

main.c

```
#include "mgos.h"
#include "mgos_my_ibeacon_lib.h"

adv_data_strct_t* create_adv_data() {

    adv_data_strct_t* beaconParam = (adv_data_strct_t*)malloc(sizeof(adv_data_strct_t));
    
    beaconParam->adv_data[0] = 2;      // Len
    beaconParam->adv_data[1] = 0x01;   // Type Flags
    beaconParam->adv_data[2] = 0x04;   // BR_EDR_NOT_SUPPORTED 0x04
    beaconParam->adv_data[3] = 26;     // Len
    beaconParam->adv_data[4] = 0xFF;   // Type
    beaconParam->adv_data[5] = 0x4C;   // Company 2 -> fake Apple 0x004C LSB
    beaconParam->adv_data[6] = 0x00;   // Company 1 MSB
    beaconParam->adv_data[7] = 0x02;   // Type Beacon
    beaconParam->adv_data[8] = 21;     // Length of Beacon Data
    beaconParam->adv_data[9] = 0x11;   // UUID 1 128-Bit (may use linux tool uuidgen or random numbers via https://www.uuidgenerator.net/)
    beaconParam->adv_data[10] = 0x22;  // UUID 2
    beaconParam->adv_data[11] = 0x33;  // UUID 3
    beaconParam->adv_data[12] = 0x53;  // UUID 4
    beaconParam->adv_data[13] = 0x32;  // UUID 5
    beaconParam->adv_data[14] = 0x6C;  // UUID 6
    beaconParam->adv_data[15] = 0x44;  // UUID 7
    beaconParam->adv_data[16] = 0x23;  // UUID 8
    beaconParam->adv_data[17] = 0xBB;  // UUID 9
    beaconParam->adv_data[18] = 0x89;  // UUID 10
    beaconParam->adv_data[19] = 0x65;  // UUID 11
    beaconParam->adv_data[20] = 0x87;  // UUID 12
    beaconParam->adv_data[21] = 0xAA;  // UUID 13
    beaconParam->adv_data[22] = 0xEE;  // UUID 14
    beaconParam->adv_data[23] = 0xEE;  // UUID 15
    beaconParam->adv_data[24] = 0x07;  // UUID 16
    beaconParam->adv_data[25] = 0x00;  // Major 1 Value
    beaconParam->adv_data[26] = 0x01;  // Major 2 Value
    beaconParam->adv_data[27] = 0x00;  // Minor 1 Value
    beaconParam->adv_data[28] = 0x02;  // Minor 2 Value
    beaconParam->adv_data[29] = 0xA0;  // Beacons TX power
    beaconParam->adv_data_len = 30;
    return beaconParam;
}

enum mgos_app_init_result mgos_app_init(void) {
  ibeacon_set_adv_data(create_adv_data());
  ibeacon_start_adv();
  return MGOS_APP_INIT_SUCCESS;
}
```
