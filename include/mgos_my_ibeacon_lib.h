#include "mgos.h"

typedef struct {
    uint8_t adv_data[31];
    uint8_t adv_data_len;
} adv_data_strct_t;

/**
 * set uuid, major, minor, measured power
 * @param  data [advertise data]
 * @return 0: success, 1: fail
 */
int ibeacon_set_adv_data(adv_data_strct_t* data);

/**
 * start advertise
 * @return 0: success, 1: fail
 */
int ibeacon_start_adv();

/**
 * stop advertise
 * @return 0: success, 1: fail
 */
int ibeacon_stop_adv();
